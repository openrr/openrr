use super::control_node::ControlNode;
use arci::gamepad::{Axis, Button, GamepadEvent};
use arci::{JointTrajectoryClient, Speaker};
use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::time::Duration;

const AXIS_GAIN: f64 = 2.0;
const JOINT_POSITION_TURBO_GAIN: f64 = 2.0;

pub struct JoyJointTeleopNode<J, S>
where
    J: JointTrajectoryClient,
    S: Speaker,
{
    joint_trajectory_client: J,
    speaker: S,
    mode: String,
    submode: String,
    joint_step: f64,
    velocity: f64,
    dof: usize,
    step_duration: Duration,
    joint_index: usize,
    is_turbo: bool,
    is_sending: bool,
}

impl<J, S> JoyJointTeleopNode<J, S>
where
    J: JointTrajectoryClient,
    S: Speaker,
{
    pub fn new(
        mode: String,
        joint_trajectory_client: J,
        joint_step: f64,
        step_duration: Duration,
        speaker: S,
    ) -> Self {
        let dof = joint_trajectory_client.joint_names().len();
        Self {
            joint_trajectory_client,
            speaker,
            mode,
            dof,
            submode: "0".to_string(),
            joint_step,
            velocity: 0.0,
            step_duration,
            joint_index: 0,
            is_turbo: false,
            is_sending: false,
        }
    }
    pub fn new_from_config(
        config: JoyJointTeleopNodeConfig,
        joint_trajectory_client: J,
        speaker: S,
    ) -> Self {
        Self::new(
            config.mode,
            joint_trajectory_client,
            config.joint_step,
            Duration::from_secs_f64(config.step_duration_secs),
            speaker,
        )
    }
}

#[async_trait]
impl<N, S> ControlNode for JoyJointTeleopNode<N, S>
where
    N: JointTrajectoryClient,
    S: Speaker,
{
    fn set_event(&mut self, event: GamepadEvent) {
        match event {
            GamepadEvent::ButtonPressed(Button::East) => {
                self.joint_index = (self.joint_index + 1) % self.dof;
                self.submode = format!("{}", self.joint_index);
                self.speaker
                    .speak(&format!("{}{}", self.mode, self.submode()));
            }
            GamepadEvent::ButtonPressed(Button::LeftTrigger2) => {
                self.is_turbo = true;
            }
            GamepadEvent::ButtonReleased(Button::LeftTrigger2) => {
                self.is_turbo = false;
            }
            GamepadEvent::ButtonPressed(Button::RightTrigger2) => {
                self.is_sending = true;
            }
            GamepadEvent::ButtonReleased(Button::RightTrigger2) => {
                self.is_sending = false;
                self.velocity = 0.0;
            }
            GamepadEvent::ButtonPressed(Button::West) => {
                self.velocity = self.joint_step;
            }
            GamepadEvent::ButtonReleased(Button::West) => {
                self.velocity = 0.0;
            }
            GamepadEvent::ButtonPressed(Button::South) => {
                self.velocity = -self.joint_step;
            }
            GamepadEvent::ButtonReleased(Button::South) => {
                self.velocity = 0.0;
            }
            GamepadEvent::AxisChanged(Axis::RightStickY, v) => {
                self.velocity = self.joint_step * v * AXIS_GAIN;
            }
            _ => {}
        }
    }
    async fn proc(&self) {
        if self.is_sending {
            let mut pos = self
                .joint_trajectory_client
                .current_joint_positions()
                .unwrap();
            pos[self.joint_index] += self.velocity
                * if self.is_turbo {
                    JOINT_POSITION_TURBO_GAIN
                } else {
                    1.0
                };
            self.joint_trajectory_client
                .send_joint_positions(pos, self.step_duration)
                .unwrap()
                .await
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
pub struct JoyJointTeleopNodeConfig {
    pub mode: String,
    #[serde(default = "default_joint_step")]
    pub joint_step: f64,
    #[serde(default = "default_step_duration_secs")]
    pub step_duration_secs: f64,
}

fn default_joint_step() -> f64 {
    0.02
}

fn default_step_duration_secs() -> f64 {
    0.1
}
