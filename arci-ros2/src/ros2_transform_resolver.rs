use std::{sync::Arc, time::Duration};

use arci::{
    nalgebra::{Quaternion, Translation3},
    Isometry3, TransformResolver, UnitQuaternion,
};
use parking_lot::Mutex;
use r2r::builtin_interfaces::msg as builtin_msg;
use tf2_r2r::{TfBuffer, TfListener};
use tracing::{debug, warn};

use crate::utils::convert_system_time_to_ros2_time;

/// TODO
pub struct Ros2TransformResolver {
    retry_rate: f64,
    max_retry: usize,
    tf_listener: Arc<Mutex<TfListener>>,
}

impl Ros2TransformResolver {
    /// TODO
    pub fn new(
        ctx: r2r::Context,
        cache_duration: Duration,
        retry_rate: f64,
        max_retry: usize,
    ) -> Self {
        let node = r2r::Node::create(ctx, "test_trasform_resolver", "arci_ros2").unwrap();

        let duration = builtin_msg::Duration {
            sec: cache_duration.as_secs() as i32,
            nanosec: cache_duration.subsec_nanos(),
        };

        Self {
            retry_rate,
            max_retry,
            tf_listener: Arc::new(Mutex::new(TfListener::new_with_buffer(
                node,
                TfBuffer::new_with_duration(duration),
            ))),
        }
    }

    /// TODO
    pub fn new_from_node(
        node: r2r::Node,
        cache_duration: Duration,
        retry_rate: f64,
        max_retry: usize,
    ) -> Self {
        let duration = builtin_msg::Duration {
            sec: cache_duration.as_secs() as i32,
            nanosec: cache_duration.subsec_nanos(),
        };

        Self {
            retry_rate,
            max_retry,
            tf_listener: Arc::new(Mutex::new(TfListener::new_with_buffer(
                node,
                TfBuffer::new_with_duration(duration),
            ))),
        }
    }
}

impl TransformResolver for Ros2TransformResolver {
    fn resolve_transformation(
        &self,
        from: &str,
        to: &str,
        time: std::time::SystemTime,
    ) -> Result<Isometry3<f64>, arci::Error> {
        const BILLION: u128 = 1_000_000_000;

        let ros_time = convert_system_time_to_ros2_time(&time);
        let wait_nanos = BILLION as u64 / self.retry_rate as u64;

        let mut last_error = None;

        for i in 0..=self.max_retry {
            if i != 0 {
                debug!("Retrying {from} -> {to} ({i} / {}) ...", self.max_retry);
            }
            let result = self
                .tf_listener
                .lock()
                .lookup_transform(from, to, ros_time.clone());
            self.tf_listener.lock().show();
            match result {
                Ok(result) => {
                    let translation = result.transform.translation;
                    let rotation = result.transform.rotation;

                    return Ok(Isometry3::from_parts(
                        Translation3::new(translation.x, translation.y, translation.z),
                        UnitQuaternion::from_quaternion(Quaternion::new(
                            rotation.w, rotation.x, rotation.y, rotation.z,
                        )),
                    ));
                }
                Err(e) => {
                    debug!("Failed to lookup_transform ({e:?})");
                    last_error = Some(e)
                }
            }
            std::thread::sleep(std::time::Duration::from_nanos(wait_nanos));
        }
        match last_error {
            Some(e) => {
                warn!("{e:?}");
                Err(anyhow::anyhow!("{e:?}").into())
            }
            None => Err(anyhow::anyhow!("Broken Logic").into()),
        }
    }
}
