use arci::{Error, Isometry3, TransformResolver};

/// Dummy TransformResolver for debug or tests.
#[derive(Debug)]
pub struct DummyTransformResolver {
    pub transformation: Isometry3<f64>,
}

impl DummyTransformResolver {
    /// Creates a new `DummyTransformResolver`.
    pub fn new() -> Self {
        Self {
            transformation: Isometry3::identity(),
        }
    }
}

impl Default for DummyTransformResolver {
    fn default() -> Self {
        Self::new()
    }
}

impl TransformResolver for DummyTransformResolver {
    fn resolve_transformation(
        &self,
        _from: &str,
        _to: &str,
        _time: std::time::SystemTime,
    ) -> Result<Isometry3<f64>, Error> {
        Ok(self.transformation)
    }
}
