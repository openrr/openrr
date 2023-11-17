use std::time::Duration;

use arci::{
    utils::{get_joint_index, move_joint_until_stop},
    DummyJointTrajectoryClient, Error, JointTrajectoryClient, TrajectoryPoint, WaitFuture,
};
use assert_approx_eq::assert_approx_eq;

struct TestJointTrajectoryClient {
    joint_names: Vec<String>,
}

impl TestJointTrajectoryClient {
    fn new() -> Self {
        Self {
            joint_names: vec![String::from("j0"), String::from("j1")],
        }
    }
}

impl JointTrajectoryClient for TestJointTrajectoryClient {
    fn joint_names(&self) -> Vec<String> {
        self.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        Ok(vec![1.0, 1.0])
    }

    fn send_joint_positions(
        &self,
        _: Vec<f64>,
        _: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        Ok(WaitFuture::ready())
    }

    fn send_joint_trajectory(&self, _: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        Ok(WaitFuture::ready())
    }
}

#[test]
fn test_get_joint_index() {
    let client = DummyJointTrajectoryClient::new(vec![String::from("j0"), String::from("j1")]);
    let j0 = get_joint_index(&client, "j0");
    assert!(j0.is_ok());
    assert_eq!(j0.unwrap(), 0);
    let j1 = get_joint_index(&client, "j1");
    assert!(j1.is_ok());
    assert_eq!(j1.unwrap(), 1);
    assert!(get_joint_index(&client, "j2").is_err());
}

#[tokio::test]
async fn test_move_joint_until_stop() {
    let client = DummyJointTrajectoryClient::new(vec![String::from("j0"), String::from("j1")]);

    let stopped_position = move_joint_until_stop(
        &client,
        "j0",
        2.0,
        Duration::from_secs_f64(1.0),
        0.01,
        Duration::from_secs_f64(1.0),
        Duration::from_secs_f64(0.1),
        Duration::from_secs_f64(0.1),
    )
    .await;
    assert!(stopped_position.is_ok());
    assert_approx_eq!(stopped_position.unwrap(), 2.0);

    let stopped_position = move_joint_until_stop(
        &client,
        "j2",
        2.0,
        Duration::from_secs_f64(1.0),
        0.01,
        Duration::from_secs_f64(1.0),
        Duration::from_secs_f64(0.1),
        Duration::from_secs_f64(0.1),
    )
    .await;
    assert!(stopped_position.is_err());

    let client = TestJointTrajectoryClient::new();

    let stopped_position = move_joint_until_stop(
        &client,
        "j0",
        2.0,
        Duration::from_secs_f64(1.0),
        0.01,
        Duration::from_secs_f64(1.0),
        Duration::from_secs_f64(0.1),
        Duration::from_secs_f64(0.1),
    )
    .await;
    assert!(stopped_position.is_ok());
    assert_approx_eq!(stopped_position.unwrap(), 1.0);
}
