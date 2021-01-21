use crate::error::Error;
use crate::traits::Navigation;
use async_trait::async_trait;
use nalgebra::{Isometry2, Vector2};
use std::sync::Mutex;

#[derive(Debug)]
pub struct DummyNavigation {
    pub current_pose: Mutex<Isometry2<f64>>,
}

impl DummyNavigation {
    pub fn new() -> Self {
        Self {
            current_pose: Mutex::new(Isometry2::new(Vector2::new(0.0, 0.0), 0.0)),
        }
    }
}

#[async_trait]
impl Navigation for DummyNavigation {
    async fn send_pose(
        &self,
        goal: Isometry2<f64>,
        _frame_id: &str,
        _timeout: std::time::Duration,
    ) -> Result<(), Error> {
        *self.current_pose.lock().unwrap() = goal;
        Ok(())
    }

    fn current_pose(&self) -> Result<Isometry2<f64>, Error> {
        Ok(self.current_pose.lock().unwrap().to_owned())
    }

    fn cancel(&self) -> Result<(), Error> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;
    #[test]
    fn test_set_get() {
        let nav = DummyNavigation::new();
        let pose0 = nav.current_pose().unwrap();
        assert_eq!(pose0, pose0.inverse()); // only identity mapping satisfies this
        assert!(tokio_test::block_on(nav.send_pose(
            Isometry2::new(Vector2::new(1.0, 2.0), 3.0),
            "",
            std::time::Duration::default(),
        ))
        .is_ok());
        let pose1 = nav.current_pose().unwrap();
        assert_eq!(pose1.translation, nalgebra::Translation2::new(1.0, 2.0));
        assert_approx_eq!(pose1.rotation.angle(), 3.0);
    }
}
