use arci::{Error, JointTrajectoryClient, TrajectoryPoint};
use async_trait::async_trait;
use log::debug;
use std::sync::Arc;
use std::time::Duration;

use crate::utils::find_nodes;

pub struct SelfCollisionChecker {
    pub using_joints: k::Chain<f64>,
    pub collision_check_robot: Arc<k::Chain<f64>>,
    pub collision_checker: openrr_planner::CollisionChecker<f64>,
    pub collision_pairs: Vec<(String, String)>,
    pub time_interpolate_rate: f64,
}

impl SelfCollisionChecker {
    pub fn new(
        joint_names: Vec<String>,
        collision_check_robot: Arc<k::Chain<f64>>,
        collision_checker: openrr_planner::CollisionChecker<f64>,
        collision_pairs: Vec<(String, String)>,
        time_interpolate_rate: f64,
    ) -> Self {
        assert!(
            time_interpolate_rate > 0.0 && time_interpolate_rate <= 1.0,
            "time_interpolate_rate must be 0.0~1.0 but {}",
            time_interpolate_rate
        );
        let using_joints =
            k::Chain::<f64>::from_nodes(find_nodes(&joint_names, &collision_check_robot).unwrap());
        Self {
            collision_check_robot,
            using_joints,
            collision_checker,
            collision_pairs,
            time_interpolate_rate,
        }
    }
    pub fn check_joint_positions(
        &self,
        current: &[f64],
        positions: &[f64],
        duration: std::time::Duration,
    ) -> Result<(), Error> {
        match openrr_planner::interpolate(
            &[current.to_vec(), positions.to_vec()],
            duration.as_secs_f64(),
            duration.as_secs_f64() * self.time_interpolate_rate,
        ) {
            Some(interpolated) => {
                debug!("interpolated len={}", interpolated.len());
                for v in interpolated {
                    self.using_joints.set_joint_positions_clamped(&v.position);
                    self.collision_check_robot.update_transforms();
                    let mut self_checker = self
                        .collision_checker
                        .check_self(&self.collision_check_robot, &self.collision_pairs);
                    if let Some(names) = self_checker.next() {
                        return Err(Error::CollisionError(names.0, names.1));
                    }
                    let mut vec_used: Vec<_> = self_checker.used_duration().iter().collect();
                    vec_used.sort_by(|a, b| b.1.cmp(a.1));
                    let sum_duration: Duration =
                        self_checker.used_duration().iter().map(|(_k, v)| v).sum();
                    debug!("total: {:?}", sum_duration);
                    debug!("detailed: {:?}", vec_used);
                }
                Ok(())
            }
            None => Err(Error::InterpolationError(
                "failed to interpolate".to_owned(),
            )),
        }
    }
    pub fn check_joint_trajectory(&self, trajectory: &[TrajectoryPoint]) -> Result<(), Error> {
        for v in trajectory {
            self.using_joints
                .set_joint_positions(&v.positions)
                .map_err(|e| Error::Other(e.into()))?;
            if let Some(names) = self
                .collision_checker
                .check_self(&self.collision_check_robot, &self.collision_pairs)
                .next()
            {
                return Err(Error::CollisionError(names.0, names.1));
            }
        }
        Ok(())
    }
}
pub struct CollisionCheckClient<T>
where
    T: JointTrajectoryClient,
{
    pub client: T,
    pub collision_checker: SelfCollisionChecker,
}

impl<T> CollisionCheckClient<T>
where
    T: JointTrajectoryClient,
{
    pub fn new(client: T, collision_checker: SelfCollisionChecker) -> Self {
        Self {
            client,
            collision_checker,
        }
    }
}

#[async_trait]
impl<T> JointTrajectoryClient for CollisionCheckClient<T>
where
    T: JointTrajectoryClient,
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
        self.collision_checker.check_joint_positions(
            &self.current_joint_positions()?,
            &positions,
            duration,
        )?;
        self.client.send_joint_positions(positions, duration).await
    }
    async fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<(), Error> {
        self.collision_checker.check_joint_trajectory(&trajectory)?;
        self.client.send_joint_trajectory(trajectory).await
    }
}
