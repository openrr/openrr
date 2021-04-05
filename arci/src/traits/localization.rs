use crate::error::Result;
use auto_impl::auto_impl;
use nalgebra::Isometry2;

#[auto_impl(Box, Arc)]
pub trait Localization: Send + Sync {
    fn current_pose(&self, frame_id: &str) -> Result<Isometry2<f64>>;
}
