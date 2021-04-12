use crate::error::Error;
use crate::{Isometry2, WaitFuture};
use auto_impl::auto_impl;

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
