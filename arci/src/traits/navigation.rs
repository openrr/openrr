use crate::error::Error;
use async_trait::async_trait;
use auto_impl::auto_impl;
use nalgebra::Isometry2;

#[async_trait]
#[auto_impl(Box, Arc)]
pub trait Navigation: Send + Sync {
    async fn send_pose(
        &self,
        goal: Isometry2<f64>,
        timeout: std::time::Duration,
    ) -> Result<(), Error>;
    fn current_pose(&self) -> Result<Isometry2<f64>, Error>;

    fn cancel(&self) -> Result<(), Error>;
}
