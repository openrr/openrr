use auto_impl::auto_impl;

use crate::{error::Error, Isometry2, WaitFuture};

#[auto_impl(Box, Arc)]
pub trait Navigation: Send + Sync {
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: std::time::Duration,
    ) -> Result<WaitFuture, Error>;

    #[allow(unused_variables)]
    fn send_goal_pose_with_thresholds(
        &self,
        goal: Isometry2<f64>,
        thresholds: [f64; 3],
        frame_id: &str,
        timeout: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        Err(anyhow::anyhow!("Not implemented").into())
    }

    fn cancel(&self) -> Result<(), Error>;
}
