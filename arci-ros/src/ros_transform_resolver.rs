use std::time::{Duration, SystemTime};

use arci::TransformResolver;

use crate::define_action_client_internal;
mod msg {
    ros_nalgebra::rosmsg_include!(
        tf2_msgs / LookupTransformActionGoal,
        tf2_msgs / LookupTransformGoal,
        tf2_msgs / LookupTransformActionResult,
        tf2_msgs / LookupTransformResult
    );
}

define_action_client_internal!(SimpleActionClient, msg::tf2_msgs, LookupTransform);

pub struct Tf2BufferServerClient {
    action_client: SimpleActionClient,
    base_time: SystemTime,
}

impl Tf2BufferServerClient {
    pub fn new(base_topic: &str, queue_size: usize, monitoring_rate: f64) -> Self {
        let action_client = SimpleActionClient::new(base_topic, queue_size, monitoring_rate);

        let base_time = if rosrust::param("/use_sim_time")
            .and_then(|v| v.get().ok())
            .unwrap_or(false)
        {
            std::time::SystemTime::now() - Duration::from_nanos(rosrust::now().nanos() as u64)
        } else {
            std::time::SystemTime::UNIX_EPOCH
        };

        Self {
            action_client,
            base_time,
        }
    }
}

impl TransformResolver for Tf2BufferServerClient {
    fn resolve_transformation(
        &self,
        from: &str,
        to: &str,
        time: std::time::SystemTime,
    ) -> Result<nalgebra::Isometry3<f64>, arci::Error> {
        const MAX_RETRY: usize = 10;
        const TIMEOUT_SEC: f64 = 10.0;
        let timeout = std::time::Duration::from_secs_f64(TIMEOUT_SEC);
        let rostime = rosrust::Time::from_nanos(
            time.duration_since(self.base_time).unwrap().as_nanos() as i64,
        );
        let goal = msg::tf2_msgs::LookupTransformGoal {
            target_frame: from.to_owned(),
            source_frame: to.to_owned(),
            source_time: rostime,
            ..Default::default()
        };

        let mut result = self
            .action_client
            .send_goal_and_wait(goal.clone(), timeout)
            .map_err(|e| anyhow::anyhow!("Failed to send_goal_and_wait : {}", e.to_string()))?;
        match result.error.error {
            msg::tf2_msgs::TF2Error::NO_ERROR => Ok(result.transform.transform.into()),
            msg::tf2_msgs::TF2Error::EXTRAPOLATION_ERROR => {
                for i in 1..=MAX_RETRY {
                    rosrust::ros_warn!("EXTRAPOLATION_ERROR : retrying {} / {} ...", i, MAX_RETRY);

                    result = self
                        .action_client
                        .send_goal_and_wait(goal.clone(), timeout)
                        .map_err(|e| {
                            anyhow::anyhow!("Failed to send_goal_and_wait : {}", e.to_string())
                        })?;
                    if result.error.error == msg::tf2_msgs::TF2Error::NO_ERROR {
                        return Ok(result.transform.transform.into());
                    }
                }
                Err(anyhow::anyhow!("TF2Error {:?}", result.error).into())
            }
            _ => Err(anyhow::anyhow!("TF2Error {:?}", result.error).into()),
        }
    }
}
