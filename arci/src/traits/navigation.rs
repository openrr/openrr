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

    fn cancel(&self) -> Result<(), Error>;
}
