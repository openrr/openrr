use std::time::{Duration, SystemTime};

use arci::{Isometry3, TransformResolver};
use nalgebra::{Quaternion, Translation3, UnitQuaternion};
use rosrust::rate;
use rustros_tf::TfListener;

pub struct RosTransformResolver {
    retry_rate: f64,
    tf_listener: TfListener,
    base_time: SystemTime,
}

impl RosTransformResolver {
    pub fn new(retry_rate: f64) -> Self {
        let base_time = if rosrust::param("/use_sim_time")
            .and_then(|v| v.get().ok())
            .unwrap_or(false)
        {
            std::time::SystemTime::now() - Duration::from_nanos(rosrust::now().nanos() as u64)
        } else {
            std::time::SystemTime::UNIX_EPOCH
        };

        Self {
            retry_rate,
            tf_listener: TfListener::new(),
            base_time,
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
        const MAX_RETRY: usize = 10;
        let ros_time = rosrust::Time::from_nanos(
            time.duration_since(self.base_time).unwrap().as_nanos() as i64,
        );
        let rate = rate(self.retry_rate);
        let mut last_error = None;
        for i in 0..=MAX_RETRY {
            if i != 0 {
                rosrust::ros_warn!("Retrying {} -> {} ({} / {}) ...", from, to, i, MAX_RETRY);
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
                Err(e) => last_error = Some(e),
            }
            rate.sleep();
        }
        match last_error {
            Some(e) => Err(anyhow::anyhow!("{:?}", e).into()),
            None => Err(anyhow::anyhow!("Broken Logic").into()),
        }
    }
}
