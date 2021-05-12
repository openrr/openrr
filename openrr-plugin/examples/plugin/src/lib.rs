use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use openrr_client::PrintSpeaker;
use openrr_plugin::{
    arci::{self, DummyMoveBase, DummyNavigation, Error, TrajectoryPoint, WaitFuture},
    Plugin, StaticJointTrajectoryClient,
};

openrr_plugin::export_plugin!(MyPlugin::new);

pub struct MyPlugin {
    joint_trajectory_client: Arc<MyJointTrajectoryClient>,
    speaker: Arc<PrintSpeaker>,
    move_base: Arc<DummyMoveBase>,
    navigation: Arc<DummyNavigation>,
}

impl MyPlugin {
    fn new(_args: Vec<String>) -> Result<Self, Error> {
        Ok(MyPlugin {
            joint_trajectory_client: Arc::new(MyJointTrajectoryClient {
                joint_names: vec!["a".to_string(), "b".to_string()],
                joint_positions: Mutex::new(vec![0.0, 0.0]),
            }),
            speaker: Arc::new(PrintSpeaker::default()),
            move_base: Arc::new(DummyMoveBase::default()),
            navigation: Arc::new(DummyNavigation::default()),
        })
    }
}

impl Plugin for MyPlugin {
    fn name(&self) -> String {
        "Example".to_string()
    }

    fn joint_trajectory_client(&self) -> Option<Arc<dyn StaticJointTrajectoryClient>> {
        Some(self.joint_trajectory_client.clone())
    }

    fn speaker(&self) -> Option<Arc<dyn arci::Speaker>> {
        Some(self.speaker.clone())
    }

    fn move_base(&self) -> Option<Arc<dyn arci::MoveBase>> {
        Some(self.move_base.clone())
    }

    fn navigation(&self) -> Option<Arc<dyn arci::Navigation>> {
        Some(self.navigation.clone())
    }
}

pub struct MyJointTrajectoryClient {
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
