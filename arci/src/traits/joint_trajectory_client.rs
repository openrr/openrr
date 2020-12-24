use crate::error::Error;
use crate::waits::CompleteCondition;
use async_trait::async_trait;
use auto_impl::auto_impl;

#[derive(Clone, Debug)]
pub struct TrajectoryPoint {
    pub positions: Vec<f64>,
    pub velocities: Option<Vec<f64>>,
    pub time_from_start: std::time::Duration,
}

impl TrajectoryPoint {
    pub fn new(positions: Vec<f64>, time_from_start: std::time::Duration) -> Self {
        Self {
            positions,
            velocities: None,
            time_from_start,
        }
    }
}

#[auto_impl(Box)]
pub trait SetCompleteCondition {
    fn set_complete_condition(&mut self, condition: Box<dyn CompleteCondition>);
}

#[async_trait]
#[auto_impl(Box, Arc)]
pub trait JointTrajectoryClient: Send + Sync {
    fn joint_names(&self) -> &[String];
    fn current_joint_positions(&self) -> Result<Vec<f64>, Error>;
    async fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<(), Error>;
    async fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<(), Error>;
}
