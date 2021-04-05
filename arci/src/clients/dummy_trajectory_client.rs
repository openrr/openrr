use crate::error::Result;
use crate::traits::{JointTrajectoryClient, TrajectoryPoint};
use async_trait::async_trait;
use std::sync::{Arc, Mutex};

/// Dummy JointTrajectoryClient for Debug or Tests
pub struct DummyJointTrajectoryClient {
    pub joint_names: Vec<String>,
    pub positions: Arc<Mutex<Vec<f64>>>,
    pub last_trajectory: Arc<Mutex<Vec<TrajectoryPoint>>>,
}

impl DummyJointTrajectoryClient {
    pub fn new(joint_names: Vec<String>) -> Self {
        let dof = joint_names.len();
        let positions = Arc::new(Mutex::new(vec![0.0; dof]));
        Self {
            joint_names,
            positions,
            last_trajectory: Arc::new(Mutex::new(Vec::new())),
        }
    }
}

#[async_trait]
impl JointTrajectoryClient for DummyJointTrajectoryClient {
    fn joint_names(&self) -> &[String] {
        &self.joint_names
    }
    fn current_joint_positions(&self) -> Result<Vec<f64>> {
        Ok(self.positions.lock().unwrap().clone())
    }
    async fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        _duration: std::time::Duration,
    ) -> Result<()> {
        *self.positions.lock().unwrap() = positions;
        Ok(())
    }
    async fn send_joint_trajectory(&self, full_trajectory: Vec<TrajectoryPoint>) -> Result<()> {
        if let Some(last_point) = full_trajectory.last() {
            *self.positions.lock().unwrap() = last_point.positions.to_owned();
        }
        *self.last_trajectory.lock().unwrap() = full_trajectory;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;
    #[test]
    fn send_and_get() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned(), "b".to_owned()]);
        assert_eq!(client.joint_names().len(), 2);
        assert_eq!(client.joint_names()[0], "a");
        assert_eq!(client.joint_names()[1], "b");
        let pos = client.current_joint_positions().unwrap();
        assert_eq!(pos.len(), 2);
        assert_approx_eq!(pos[0], 0.0);
        assert_approx_eq!(pos[1], 0.0);
        assert!(tokio_test::block_on(
            client.send_joint_positions(vec![1.0, 2.0], std::time::Duration::from_secs(1)),
        )
        .is_ok());
        let pos2 = client.current_joint_positions().unwrap();
        assert_eq!(pos2.len(), 2);
        assert_approx_eq!(pos2[0], 1.0);
        assert_approx_eq!(pos2[1], 2.0);
    }
    #[test]
    fn trajectory() {
        let client = DummyJointTrajectoryClient::new(vec!["aa".to_owned(), "bb".to_owned()]);
        assert_eq!(client.last_trajectory.lock().unwrap().len(), 0);
        let result = client.send_joint_trajectory(vec![
            TrajectoryPoint::new(vec![1.0, -1.0], std::time::Duration::from_secs(1)),
            TrajectoryPoint::new(vec![2.0, -3.0], std::time::Duration::from_secs(2)),
        ]);
        assert!(tokio_test::block_on(result).is_ok());
        assert_eq!(client.last_trajectory.lock().unwrap().len(), 2);

        let pos = client.current_joint_positions().unwrap();
        assert_eq!(pos.len(), 2);
        assert_approx_eq!(pos[0], 2.0);
        assert_approx_eq!(pos[1], -3.0);
    }
}
