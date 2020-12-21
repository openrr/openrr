use crate::error::Error;
use crate::traits::{JointTrajectoryClient, TrajectoryPoint};
use async_trait::async_trait;
use log::debug;

pub struct JointVelocityLimiter<C>
where
    C: JointTrajectoryClient + Send + Sync,
{
    client: C,
    velocity_limits: Vec<f64>,
}

impl<C> JointVelocityLimiter<C>
where
    C: JointTrajectoryClient + Send + Sync,
{
    pub fn new(client: C, velocity_limits: Vec<f64>) -> Self {
        assert!(client.joint_names().len() == velocity_limits.len());
        Self {
            client,
            velocity_limits,
        }
    }
}

#[async_trait]
impl<C> JointTrajectoryClient for JointVelocityLimiter<C>
where
    C: JointTrajectoryClient + Send + Sync,
{
    fn joint_names(&self) -> &[String] {
        self.client.joint_names()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        self.client.current_joint_positions()
    }

    async fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<(), Error> {
        Ok(self
            .send_joint_trajectory(vec![TrajectoryPoint {
                positions,
                velocities: None,
                time_from_start: duration,
            }])
            .await?)
    }

    async fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<(), Error> {
        let mut prev_positions = self.current_joint_positions()?;

        let mut limited_trajectory = vec![];
        let mut limited_duration_from_start = std::time::Duration::from_secs(0);
        let mut original_duration_from_start = std::time::Duration::from_secs(0);
        for (sequence_index, original_trajectory_point) in trajectory.iter().enumerate() {
            let mut limited_duration_from_prev = std::time::Duration::from_secs(0);
            let mut dominant_joint_index = 0;
            for (joint_index, prev_position) in prev_positions.iter().enumerate() {
                let single_duration = std::time::Duration::from_secs_f64(
                    (prev_position - original_trajectory_point.positions[joint_index]).abs()
                        / self.velocity_limits[joint_index],
                );
                limited_duration_from_prev = if single_duration > limited_duration_from_prev {
                    dominant_joint_index = joint_index;
                    single_duration
                } else {
                    limited_duration_from_prev
                }
            }
            let original_duration_from_prev =
                original_trajectory_point.time_from_start - original_duration_from_start;
            original_duration_from_start = original_trajectory_point.time_from_start;

            let use_limited = limited_duration_from_prev > original_duration_from_prev;
            let selected_duration = if use_limited {
                limited_duration_from_prev
            } else {
                original_duration_from_prev
            };
            limited_duration_from_start += selected_duration;
            limited_trajectory.push(TrajectoryPoint {
                positions: original_trajectory_point.positions.clone(),
                velocities: original_trajectory_point.velocities.clone(),
                time_from_start: limited_duration_from_start,
            });
            prev_positions = original_trajectory_point.positions.clone();
            debug!(
                "Sequence{} dominant joint_index {} duration limited : {:?}{} original : {:?}{}",
                sequence_index,
                dominant_joint_index,
                limited_duration_from_prev,
                if use_limited { "(O)" } else { "" },
                original_duration_from_prev,
                if use_limited { "" } else { "(O)" }
            );
        }

        debug!("OriginalTrajectory {:?}", trajectory);
        debug!("LimitedTrajectory {:?}", limited_trajectory);

        Ok(self
            .client
            .send_joint_trajectory(limited_trajectory)
            .await?)
    }
}
