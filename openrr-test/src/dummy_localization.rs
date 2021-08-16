use arci::{Error, Isometry2, Localization, Vector2};

/// Dummy Localization for debug or tests.
#[derive(Debug)]
pub struct DummyLocalization {
    pub current_pose: Isometry2<f64>,
}

impl DummyLocalization {
    /// Creates a new `DummyLocalization`.
    pub fn new() -> Self {
        Self {
            current_pose: Isometry2::new(Vector2::new(0.0, 0.0), 0.0),
        }
    }
}

impl Default for DummyLocalization {
    fn default() -> Self {
        Self::new()
    }
}

impl Localization for DummyLocalization {
    fn current_pose(&self, _frame_id: &str) -> Result<Isometry2<f64>, Error> {
        Ok(self.current_pose)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_get() {
        let loc = DummyLocalization::new();
        let current_pose = loc.current_pose("").unwrap();
        assert_eq!(current_pose, current_pose.inverse()); // only identity mapping satisfies this
    }
}
