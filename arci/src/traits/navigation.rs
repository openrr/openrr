use crate::error::Error;
use crate::{Isometry2, Wait};
use auto_impl::auto_impl;

#[auto_impl(Box, Arc)]
pub trait Navigation: Send + Sync {
    fn move_to(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: std::time::Duration,
    ) -> Result<Wait, Error>;

    fn cancel(&self) -> Result<(), Error>;
}
