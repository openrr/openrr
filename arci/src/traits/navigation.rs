use crate::error::Error;
use crate::Isometry2;
use async_trait::async_trait;
use auto_impl::auto_impl;

#[async_trait]
#[auto_impl(Box, Arc)]
pub trait Navigation: Send + Sync {
    async fn send_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: std::time::Duration,
    ) -> Result<(), Error>;

    fn cancel(&self) -> Result<(), Error>;
}
