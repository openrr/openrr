use crate::error::Error;
use auto_impl::auto_impl;

#[auto_impl(Box, Rc, Arc)]
pub trait TransformResolver {
    fn resolve_transformation(
        &self,
        from: &str,
        to: &str,
        time: std::time::SystemTime,
    ) -> Result<nalgebra::Isometry3<f64>, Error>;
}
