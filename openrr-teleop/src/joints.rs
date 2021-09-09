use std::time::Duration;

use arci::{
    gamepad::{Axis, Button, GamepadEvent},
    JointTrajectoryClient, Speaker,
};
use async_trait::async_trait;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use super::control_node::ControlNode;

const AXIS_GAIN: f64 = 2.0;
const JOINT_POSITION_TURBO_GAIN: f64 = 2.0;

struct JoyJointTeleopNodeInner {
    submode: String,
    velocity: f64,
    dof: usize,
    joint_index: usize,
    joint_step: f64,
    is_turbo: bool,
    is_sending: bool,
}

impl JoyJointTeleopNodeInner {
    fn new(joint_step: f64, dof: usize) -> Self {
        Self {
            dof,
            submode: "0".to_string(),
            velocity: 0.0,
            joint_index: 0,
            joint_step,
            is_turbo: false,
            is_sending: false,
        }
    }

    fn set_event(&mut self, event: GamepadEvent) -> Option<&str> {
        match event {
            GamepadEvent::ButtonPressed(Button::East) => {
                self.joint_index = (self.joint_index + 1) % self.dof;
                self.submode = format!("{}", self.joint_index);
                return Some(&self.submode);
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
            GamepadEvent::Disconnected => {
                self.is_sending = false;
                self.is_turbo = false;
                self.velocity = 0.0;
            }
            _ => {}
        }
        None
    }

    fn get_target_positions(&self, mut current_positions: Vec<f64>) -> Vec<f64> {
        current_positions[self.joint_index] += self.velocity
            * if self.is_turbo {
                JOINT_POSITION_TURBO_GAIN
            } else {
                1.0
            };
        current_positions
    }
}

pub struct JoyJointTeleopNode<J, S>
where
    J: JointTrajectoryClient,
    S: Speaker,
{
    joint_trajectory_client: J,
    speaker: S,
    mode: String,
    step_duration: Duration,
    inner: JoyJointTeleopNodeInner,
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
            step_duration,
            inner: JoyJointTeleopNodeInner::new(joint_step, dof),
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
        if let Some(submode) = self.inner.set_event(event) {
            // do not wait
            let _ = self
                .speaker
                .speak(&format!("{}{}", self.mode, submode))
                .unwrap();
        }
    }

    async fn proc(&self) {
        if self.inner.is_sending {
            let pos = self
                .joint_trajectory_client
                .current_joint_positions()
                .unwrap();
            let pos = self.inner.get_target_positions(pos);
            // do not wait
            let _ = self
                .joint_trajectory_client
                .send_joint_positions(pos, self.step_duration)
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
pub struct JoyJointTeleopNodeConfig {
    pub mode: String,
    #[serde(default = "default_joint_step")]
    pub joint_step: f64,
    #[serde(default = "default_step_duration_secs")]
    pub step_duration_secs: f64,
}

const fn default_joint_step() -> f64 {
    0.02
}

const fn default_step_duration_secs() -> f64 {
    0.1
}

#[cfg(test)]
mod tests {
    use assert_approx_eq::*;

    use super::*;

    #[test]
    fn test_default_joint_step() {
        let def = default_joint_step();

        assert_approx_eq!(def, 0.02_f64);
    }

    #[test]
    fn test_default_step_duration_secs() {
        let def = default_step_duration_secs();

        assert_approx_eq!(def, 0.1_f64);
    }
}
