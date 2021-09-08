use std::{collections::HashMap, time::Duration};

use arci::{
    gamepad::{Button, GamepadEvent},
    JointTrajectoryClient, Speaker,
};
use async_trait::async_trait;
use openrr_client::JointsPose;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::ControlNode;

struct JointsPoseSenderInner {
    joints_poses: Vec<JointsPose>,
    submode: String,
    pose_index: usize,
    is_trigger_holding: bool,
    is_sending: bool,
}

impl JointsPoseSenderInner {
    fn new(joints_poses: Vec<JointsPose>) -> Self {
        Self {
            submode: format!(
                " {} {}",
                joints_poses[0].client_name, joints_poses[0].pose_name
            ),
            joints_poses,
            pose_index: 0,
            is_trigger_holding: false,
            is_sending: false,
        }
    }

    fn set_event(&mut self, event: arci::gamepad::GamepadEvent) -> Option<&str> {
        match event {
            GamepadEvent::ButtonPressed(Button::East) => {
                self.pose_index = (self.pose_index + 1) % self.joints_poses.len();
                let joints_pose = &self.joints_poses[self.pose_index];
                self.submode = format!(" {} {}", joints_pose.client_name, joints_pose.pose_name);
                return Some(&self.submode);
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
            GamepadEvent::Disconnected => {
                self.is_trigger_holding = false;
                self.is_sending = false;
            }
            _ => {}
        }
        None
    }

    fn get_target_name_positions(&self) -> (String, Vec<f64>) {
        let joints_pose = &self.joints_poses[self.pose_index];
        (
            joints_pose.client_name.to_owned(),
            joints_pose.positions.to_owned(),
        )
    }
}
pub struct JointsPoseSender<S, J>
where
    S: Speaker,
    J: JointTrajectoryClient,
{
    mode: String,
    joint_trajectory_clients: HashMap<String, J>,
    speaker: S,
    duration: Duration,
    inner: JointsPoseSenderInner,
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
            joint_trajectory_clients,
            speaker,
            duration,
            inner: JointsPoseSenderInner::new(joints_poses),
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
        if let Some(submode) = self.inner.set_event(event) {
            // do not wait
            let _ = self
                .speaker
                .speak(&format!("{}{}", self.mode, submode))
                .unwrap();
        }
    }

    async fn proc(&self) {
        let (name, target) = self.inner.get_target_name_positions();
        let client = self.joint_trajectory_clients.get(&name).unwrap();
        if self.inner.is_sending && self.inner.is_trigger_holding {
            let _ = client.send_joint_positions(target, self.duration).unwrap();
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
        &self.inner.submode
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
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
