use std::time::Duration;

use arci::{
    nalgebra::{Quaternion, Translation3},
    Isometry3, TransformResolver, UnitQuaternion,
};
use tf_ros2::{TfBuffer, TfListener};
use tracing::{debug, warn};

use crate::{msg::tf2_msgs::TFMessage, utils::convert_system_time_to_ros2_time, Node};

/// `arci::TransformResolver` implementation for ROS2.
pub struct Ros2TransformResolver {
    retry_rate: f64,
    max_retry: usize,
    tf_listener: TfListener,
    // keep not to be dropped
    _node: Node,
}

impl Ros2TransformResolver {
    /// Creates a new `Ros2TransformResolver`.
    #[track_caller]
    pub fn new(node: Node, cache_duration: Duration, retry_rate: f64, max_retry: usize) -> Self {
        let duration = ros2_client::builtin_interfaces::Duration {
            sec: cache_duration.as_secs() as i32,
            nanosec: cache_duration.subsec_nanos(),
        };

        let tf_topic = node.create_topic::<TFMessage>("/tf").unwrap();
        let tf_static_topic = node.create_topic::<TFMessage>("/tf_static").unwrap();
        let tf_listener = TfListener::new_with_buffer(
            &mut node.inner.node.lock().unwrap(),
            &tf_topic,
            &tf_static_topic,
            TfBuffer::new_with_duration(duration),
        );
        Self {
            retry_rate,
            max_retry,
            tf_listener,
            _node: node,
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
            let result = self.tf_listener.lookup_transform(from, to, ros_time);
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
            std::thread::sleep(Duration::from_nanos(wait_nanos));
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
