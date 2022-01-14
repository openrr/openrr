use std::time::Duration;

use arci::{Isometry3, TransformResolver};
use nalgebra::{Quaternion, Translation3, UnitQuaternion};
use rosrust::rate;
use tf_rosrust::{TfBuffer, TfListener};
use tracing::{debug, warn};

use crate::convert_system_time_to_ros_time;

pub struct RosTransformResolver {
    retry_rate: f64,
    max_retry: usize,
    tf_listener: TfListener,
}

impl RosTransformResolver {
    pub fn new(cache_duration: Duration, retry_rate: f64, max_retry: usize) -> Self {
        Self {
            retry_rate,
            max_retry,
            tf_listener: TfListener::new_with_buffer(TfBuffer::new_with_duration(
                cache_duration.into(),
            )),
        }
    }
}

impl TransformResolver for RosTransformResolver {
    fn resolve_transformation(
        &self,
        from: &str,
        to: &str,
        time: std::time::SystemTime,
    ) -> Result<nalgebra::Isometry3<f64>, arci::Error> {
        let ros_time = convert_system_time_to_ros_time(&time);
        let rate = rate(self.retry_rate);
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
            rate.sleep();
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
