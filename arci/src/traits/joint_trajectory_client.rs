use auto_impl::auto_impl;
use serde::{Deserialize, Serialize};

use crate::{
    error::Error,
    waits::{CompleteCondition, WaitFuture},
};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    pub positions: Vec<f64>,
    pub velocities: Option<Vec<f64>>,
    pub time_from_start: std::time::Duration,
}

impl TrajectoryPoint {
    pub fn new(positions: Vec<f64>, time_from_start: std::time::Duration) -> Self {
        Self {
            positions,
            velocities: None,
            time_from_start,
        }
    }
}

#[auto_impl(Box)]
pub trait SetCompleteCondition {
    fn set_complete_condition(&mut self, condition: Box<dyn CompleteCondition>);
}

#[auto_impl(Box, Arc)]
pub trait JointTrajectoryClient: Send + Sync {
    /// Returns names of joints that this client handles.
    fn joint_names(&self) -> Vec<String>;

    /// Returns the current joint positions.
    fn current_joint_positions(&self) -> Result<Vec<f64>, Error>;

    /// Send the specified joint positions and returns a future that waits until
    /// complete the move joints.
    ///
    /// # Implementation
    ///
    /// The returned future is expected to behave similarly to
    /// [`std::thread::JoinHandle`] and [`tokio::task::JoinHandle`]:
    ///
    /// - Can wait for the operation to complete by `.await`.
    /// - The operation does not end even if it is dropped.
    ///
    /// If the operation may block the current thread for an extended period of
    /// time, consider [spawning a thread to running blocking
    /// operations](https://docs.rs/tokio/1/tokio/index.html#cpu-bound-tasks-and-blocking-code).
    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<WaitFuture, Error>;

    /// Send the specified joint trajectory and returns a future that waits until
    /// complete the move joints.
    ///
    /// # Implementation
    ///
    /// See the "Implementation" section of the
    /// [`send_joint_positions`](Self::send_joint_positions) method.
    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error>;
}

#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;

    use super::*;

    #[test]
    fn test_trajectory_point() {
        let mut tp = TrajectoryPoint::new(vec![1.0, -1.0], std::time::Duration::from_secs(1));
        assert_approx_eq!(tp.positions[0], 1.0);
        assert_approx_eq!(tp.positions[1], -1.0);
        assert!(tp.velocities.is_none());
        assert_eq!(tp.time_from_start, std::time::Duration::from_secs(1));
        tp.positions = vec![-1.0, 1.0];
        tp.velocities = Some(vec![1.0, -1.0]);
        tp.time_from_start = std::time::Duration::from_secs(2);
        assert_approx_eq!(tp.positions[0], -1.0);
        assert_approx_eq!(tp.positions[1], 1.0);
        let vels = tp.velocities.unwrap();
        assert_approx_eq!(vels[0], 1.0);
        assert_approx_eq!(vels[1], -1.0);
        assert_eq!(tp.time_from_start, std::time::Duration::from_secs(2));
    }

    #[test]
    fn test_trajectory_point_debug() {
        let tp = TrajectoryPoint::new(vec![1.0, -1.0], std::time::Duration::from_secs(1));
        assert_eq!(
            format!("{tp:?}"),
            "TrajectoryPoint { positions: [1.0, -1.0], velocities: None, time_from_start: 1s }"
        );
    }

    #[test]
    fn test_trajectory_point_clone() {
        let tp1 = TrajectoryPoint::new(vec![1.0, -1.0], std::time::Duration::from_secs(1));
        let tp2 = tp1.clone();
        assert_approx_eq!(tp2.positions[0], 1.0);
        assert_approx_eq!(tp2.positions[1], -1.0);
        assert_approx_eq!(tp1.positions[0], 1.0);
        assert_approx_eq!(tp1.positions[1], -1.0);
    }
}
