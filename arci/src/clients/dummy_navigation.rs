use crate::traits::Navigation;
use crate::{error::Error, DummyWait, Wait};
use nalgebra::{Isometry2, Vector2};
use std::sync::Mutex;

#[derive(Debug)]
pub struct DummyNavigation {
    pub goal_pose: Mutex<Isometry2<f64>>,
}

impl DummyNavigation {
    pub fn new() -> Self {
        Self {
            goal_pose: Mutex::new(Isometry2::new(Vector2::new(0.0, 0.0), 0.0)),
        }
    }

    pub fn current_goal_pose(&self) -> Result<Isometry2<f64>, Error> {
        Ok(self.goal_pose.lock().unwrap().to_owned())
    }
}

impl Default for DummyNavigation {
    fn default() -> Self {
        Self::new()
    }
}

impl Navigation for DummyNavigation {
    fn move_to(
        &self,
        goal: Isometry2<f64>,
        _frame_id: &str,
        _timeout: std::time::Duration,
    ) -> Result<Wait, Error> {
        *self.goal_pose.lock().unwrap() = goal;
        Ok(DummyWait::new_boxed())
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
    fn test_set() {
        let nav = DummyNavigation::new();
        assert!(nav
            .move_to(
                Isometry2::new(Vector2::new(1.0, 2.0), 3.0),
                "",
                std::time::Duration::default(),
            )
            .unwrap()
            .wait()
            .is_ok());

        let current_goal_pose = nav.current_goal_pose().unwrap();
        assert_approx_eq!(current_goal_pose.translation.x, 1.0);
        assert_approx_eq!(current_goal_pose.translation.y, 2.0);
        assert_approx_eq!(current_goal_pose.rotation.angle(), 3.0);
    }
}
