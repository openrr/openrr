use std::{sync::Mutex, time::Duration};

use arci::{DummyMoveBase, DummyNavigation, Error, TrajectoryPoint, WaitFuture};
use openrr_client::PrintSpeaker;
use openrr_plugin::{Plugin, StaticJointTrajectoryClient};
use serde::Deserialize;

openrr_plugin::export_plugin!(MyPlugin);

pub struct MyPlugin;

impl Plugin for MyPlugin {
    fn name(&self) -> String {
        "Example".into()
    }

    fn new_joint_trajectory_client(
        &self,
        args: String,
    ) -> Option<Box<dyn StaticJointTrajectoryClient>> {
        let config: MyClientConfig = serde_json::from_str(&args).ok()?;
        let dof = config.joint_names.len();
        Some(Box::new(MyJointTrajectoryClient {
            joint_names: config.joint_names,
            joint_positions: Mutex::new(vec![0.0; dof]),
        }))
    }

    fn new_speaker(&self, _args: String) -> Option<Box<dyn arci::Speaker>> {
        Some(Box::new(PrintSpeaker::default()))
    }

    fn new_move_base(&self, _args: String) -> Option<Box<dyn arci::MoveBase>> {
        Some(Box::new(DummyMoveBase::default()))
    }

    fn new_navigation(&self, _args: String) -> Option<Box<dyn arci::Navigation>> {
        Some(Box::new(DummyNavigation::default()))
    }
}

#[derive(Deserialize)]
struct MyClientConfig {
    joint_names: Vec<String>,
}

struct MyJointTrajectoryClient {
    joint_names: Vec<String>,
    joint_positions: Mutex<Vec<f64>>,
}

impl StaticJointTrajectoryClient for MyJointTrajectoryClient {
    fn joint_names(&self) -> Vec<String> {
        self.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        Ok(self.joint_positions.lock().unwrap().clone())
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        _duration: Duration,
    ) -> Result<WaitFuture<'static>, Error> {
        *self.joint_positions.lock().unwrap() = positions;
        Ok(WaitFuture::new(async move { async { Ok(()) }.await }))
    }

    fn send_joint_trajectory(
        &self,
        _trajectory: Vec<TrajectoryPoint>,
    ) -> Result<WaitFuture<'static>, Error> {
        std::process::abort()
    }
}
