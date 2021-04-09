use crate::ControlNode;
use arci::{
    gamepad::{Button, GamepadEvent},
    JointTrajectoryClient, Speaker,
};
use async_trait::async_trait;
use openrr_client::JointsPose;
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, time::Duration};

pub struct JointsPoseSender<S, J>
where
    S: Speaker,
    J: JointTrajectoryClient,
{
    mode: String,
    joints_poses: Vec<JointsPose>,
    joint_trajectory_clients: HashMap<String, J>,
    speaker: S,
    submode: String,
    pose_index: usize,
    is_trigger_holding: bool,
    is_sending: bool,
    duration: Duration,
}

impl<S, J> JointsPoseSender<S, J>
where
    S: Speaker,
    J: JointTrajectoryClient,
{
    pub fn new(
        mode: String,
        joints_poses: Vec<JointsPose>,
        joint_trajectory_clients: HashMap<String, J>,
        speaker: S,
        duration: Duration,
    ) -> Self {
        Self {
            mode,
            submode: format!(
                " {} {}",
                joints_poses[0].client_name, joints_poses[0].pose_name
            ),
            joints_poses,
            joint_trajectory_clients,
            speaker,
            pose_index: 0,
            is_trigger_holding: false,
            is_sending: false,
            duration,
        }
    }
    pub fn new_from_config(
        config: JointsPoseSenderConfig,
        joints_poses: Vec<JointsPose>,
        joint_trajectory_clients: HashMap<String, J>,
        speaker: S,
    ) -> Self {
        Self::new(
            config.mode,
            joints_poses,
            joint_trajectory_clients,
            speaker,
            Duration::from_secs_f64(config.duration_secs),
        )
    }
}

#[async_trait]
impl<S, J> ControlNode for JointsPoseSender<S, J>
where
    S: Speaker,
    J: JointTrajectoryClient,
{
    fn set_event(&mut self, event: arci::gamepad::GamepadEvent) {
        match event {
            GamepadEvent::ButtonPressed(Button::East) => {
                self.pose_index = (self.pose_index + 1) % self.joints_poses.len();
                let joints_pose = &self.joints_poses[self.pose_index];
                self.submode = format!(" {} {}", joints_pose.client_name, joints_pose.pose_name);
                // do not wait
                let _ = self
                    .speaker
                    .speak(&format!("{}{}", self.mode, self.submode()))
                    .unwrap();
            }
            GamepadEvent::ButtonPressed(Button::RightTrigger2) => {
                self.is_trigger_holding = true;
            }
            GamepadEvent::ButtonReleased(Button::RightTrigger2) => {
                self.is_trigger_holding = false;
                self.is_sending = false;
            }
            GamepadEvent::ButtonPressed(Button::West) => {
                self.is_sending = true;
            }
            GamepadEvent::ButtonReleased(Button::West) => {
                self.is_sending = false;
            }
            _ => {}
        }
    }
    async fn proc(&self) {
        let joints_pose = &self.joints_poses[self.pose_index];
        let client = self
            .joint_trajectory_clients
            .get(&joints_pose.client_name)
            .unwrap();
        if self.is_sending && self.is_trigger_holding {
            let _ = client
                .send_joint_positions(joints_pose.positions.to_owned(), self.duration)
                .unwrap();
        } else {
            let _ = client
                .send_joint_positions(client.current_joint_positions().unwrap(), self.duration)
                .unwrap();
        }
    }
    fn mode(&self) -> &str {
        &self.mode
    }
    fn submode(&self) -> &str {
        &self.submode
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct JointsPoseSenderConfig {
    #[serde(default = "default_mode")]
    pub mode: String,
    #[serde(default = "default_duration_secs")]
    pub duration_secs: f64,
}

fn default_mode() -> String {
    "pose".to_string()
}

fn default_duration_secs() -> f64 {
    2.0
}
