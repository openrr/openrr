use nalgebra::{Isometry2, Vector2};
use parking_lot::Mutex;

use crate::{error::Error, traits::Navigation, WaitFuture};

/// Dummy Navigation for debug or tests.
#[derive(Debug)]
pub struct DummyNavigation {
    pub goal_pose: Mutex<Isometry2<f64>>,
    canceled: Mutex<bool>,
}

impl DummyNavigation {
    /// Creates a new `DummyNavigation`.
    pub fn new() -> Self {
        Self {
            goal_pose: Mutex::new(Isometry2::new(Vector2::new(0.0, 0.0), 0.0)),
            canceled: Mutex::default(),
        }
    }

    pub fn current_goal_pose(&self) -> Result<Isometry2<f64>, Error> {
        Ok(self.goal_pose.lock().to_owned())
    }

    pub fn is_canceled(&self) -> bool {
        *self.canceled.lock()
    }
}

impl Default for DummyNavigation {
    fn default() -> Self {
        Self::new()
    }
}

impl Navigation for DummyNavigation {
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        _frame_id: &str,
        _timeout: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        *self.canceled.lock() = false;
        *self.goal_pose.lock() = goal;
        Ok(WaitFuture::ready())
    }

    fn cancel(&self) -> Result<(), Error> {
        *self.canceled.lock() = true;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;

    use super::*;

    #[tokio::test]
    async fn test_set() {
        let nav = DummyNavigation::new();
        assert!(nav
            .send_goal_pose(
                Isometry2::new(Vector2::new(1.0, 2.0), 3.0),
                "",
                std::time::Duration::default(),
            )
            .unwrap()
            .await
            .is_ok());

        let current_goal_pose = nav.current_goal_pose().unwrap();
        assert_approx_eq!(current_goal_pose.translation.x, 1.0);
        assert_approx_eq!(current_goal_pose.translation.y, 2.0);
        assert_approx_eq!(current_goal_pose.rotation.angle(), 3.0);
    }

    #[test]
    fn test_set_no_wait() {
        let nav = DummyNavigation::new();
        let _ = nav
            .send_goal_pose(
                Isometry2::new(Vector2::new(1.0, 2.0), 3.0),
                "",
                std::time::Duration::default(),
            )
            .unwrap();

        let current_goal_pose = nav.current_goal_pose().unwrap();
        assert_approx_eq!(current_goal_pose.translation.x, 1.0);
        assert_approx_eq!(current_goal_pose.translation.y, 2.0);
        assert_approx_eq!(current_goal_pose.rotation.angle(), 3.0);
    }
}
