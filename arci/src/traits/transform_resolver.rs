use crate::error::Error;
use crate::Isometry3;
use auto_impl::auto_impl;

#[auto_impl(Box, Arc)]
pub trait TransformResolver: Send + Sync {
    fn resolve_transformation(
        &self,
        from: &str,
        to: &str,
        time: std::time::SystemTime,
    ) -> Result<Isometry3<f64>, Error>;
}
