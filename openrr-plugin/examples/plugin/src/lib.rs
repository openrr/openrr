use std::time::Duration;

use arci::{DummyLocalization, DummyMoveBase, DummyNavigation, Error, TrajectoryPoint, WaitFuture};
use openrr_client::PrintSpeaker;
use openrr_plugin::Plugin;
use parking_lot::Mutex;
use serde::Deserialize;

openrr_plugin::export_plugin!(MyPlugin);

pub struct MyPlugin;

impl Plugin for MyPlugin {
    fn new_joint_trajectory_client(
        &self,
        args: String,
    ) -> Result<Option<Box<dyn arci::JointTrajectoryClient>>, arci::Error> {
        let config: MyClientConfig =
            serde_json::from_str(&args).map_err(|e| arci::Error::Other(e.into()))?;
        let dof = config.joint_names.len();
        Ok(Some(Box::new(MyJointTrajectoryClient {
            joint_names: config.joint_names,
            joint_positions: Mutex::new(vec![0.0; dof]),
        })))
    }

    fn new_speaker(&self, _args: String) -> Result<Option<Box<dyn arci::Speaker>>, arci::Error> {
        Ok(Some(Box::<PrintSpeaker>::default()))
    }

    fn new_move_base(&self, _args: String) -> Result<Option<Box<dyn arci::MoveBase>>, arci::Error> {
        Ok(Some(Box::<DummyMoveBase>::default()))
    }

    fn new_navigation(
        &self,
        _args: String,
    ) -> Result<Option<Box<dyn arci::Navigation>>, arci::Error> {
        Ok(Some(Box::<DummyNavigation>::default()))
    }

    fn new_localization(
        &self,
        _args: String,
    ) -> Result<Option<Box<dyn arci::Localization>>, arci::Error> {
        Ok(Some(Box::<DummyLocalization>::default()))
    }

    #[cfg(unix)]
    fn new_gamepad(&self, _args: String) -> Result<Option<Box<dyn arci::Gamepad>>, arci::Error> {
        Ok(Some(
            Box::new(arci_gamepad_keyboard::KeyboardGamepad::new()),
        ))
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

impl arci::JointTrajectoryClient for MyJointTrajectoryClient {
    fn joint_names(&self) -> Vec<String> {
        self.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        Ok(self.joint_positions.lock().clone())
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture, Error> {
        println!("positions = {positions:?}, duration = {duration:?}");
        *self.joint_positions.lock() = positions;
        Ok(WaitFuture::new(async { Ok(()) }))
    }

    fn send_joint_trajectory(
        &self,
        _trajectory: Vec<TrajectoryPoint>,
    ) -> Result<WaitFuture, Error> {
        // panic across the FFI boundary will be converted to abort by abi_stable.
        panic!()
    }
}
