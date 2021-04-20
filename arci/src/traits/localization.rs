use auto_impl::auto_impl;
use nalgebra::Isometry2;

use crate::error::Error;

#[auto_impl(Box, Arc)]
pub trait Localization: Send + Sync {
    fn current_pose(&self, frame_id: &str) -> Result<Isometry2<f64>, Error>;
}
