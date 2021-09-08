use std::{sync::Arc, time::Duration};

use arci::{
    gamepad::{Axis, Button, GamepadEvent},
    JointTrajectoryClient, Speaker,
};
use async_trait::async_trait;
use k::{Translation3, Vector3};
use openrr_client::IkSolverWithChain;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use super::control_node::ControlNode;

const IK_POSITION_TURBO_GAIN: f64 = 2.0;

struct IkNodeInner {
    linear_velocity: Vector3<f64>,
    angular_velocity: Vector3<f64>,
    move_step_linear: [f64; 3],
    move_step_angular: [f64; 3],
    is_turbo: bool,
    is_sending: bool,
}

impl IkNodeInner {
    fn new(move_step_linear: [f64; 3], move_step_angular: [f64; 3]) -> Self {
        Self {
            linear_velocity: Vector3::new(0.0, 0.0, 0.0),
            angular_velocity: Vector3::new(0.0, 0.0, 0.0),
            move_step_linear,
            move_step_angular,
            is_turbo: false,
            is_sending: false,
        }
    }

    fn set_event(&mut self, event: GamepadEvent) {
        match event {
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
                self.clear_velocity();
            }
            GamepadEvent::ButtonPressed(Button::South) => {
                self.linear_velocity.z = -self.move_step_linear[2];
            }
            GamepadEvent::ButtonReleased(Button::South) => {
                self.linear_velocity.z = 0.0;
            }
            GamepadEvent::ButtonPressed(Button::West) => {
                self.linear_velocity.z = self.move_step_linear[2];
            }
            GamepadEvent::ButtonReleased(Button::West) => {
                self.linear_velocity.z = 0.0;
            }
            GamepadEvent::AxisChanged(Axis::RightStickY, v) => {
                self.linear_velocity.x = self.move_step_linear[0] * v;
            }
            GamepadEvent::AxisChanged(Axis::RightStickX, v) => {
                self.linear_velocity.y = self.move_step_linear[1] * v;
            }
            GamepadEvent::AxisChanged(Axis::LeftStickX, v) => {
                self.angular_velocity.x = -self.move_step_angular[0] * v;
            }
            GamepadEvent::AxisChanged(Axis::LeftStickY, v) => {
                self.angular_velocity.y = self.move_step_angular[1] * v;
            }
            GamepadEvent::ButtonPressed(Button::DPadRight) => {
                self.angular_velocity.z = -self.move_step_angular[2];
            }
            GamepadEvent::ButtonReleased(Button::DPadRight) => {
                self.angular_velocity.z = 0.0;
            }
            GamepadEvent::ButtonPressed(Button::DPadLeft) => {
                self.angular_velocity.z = self.move_step_angular[2];
            }
            GamepadEvent::ButtonReleased(Button::DPadLeft) => {
                self.angular_velocity.z = 0.0;
            }
            GamepadEvent::Disconnected => {
                self.is_sending = false;
                self.is_turbo = false;
                self.clear_velocity();
            }
            _ => {}
        }
    }

    fn clear_velocity(&mut self) {
        self.linear_velocity.x = 0.0;
        self.linear_velocity.y = 0.0;
        self.linear_velocity.z = 0.0;
        self.angular_velocity.x = 0.0;
        self.angular_velocity.y = 0.0;
        self.angular_velocity.z = 0.0;
    }

    fn get_linear_velocity(&self) -> Vector3<f64> {
        self.linear_velocity
            * if self.is_turbo {
                IK_POSITION_TURBO_GAIN
            } else {
                1.0
            }
    }
}

pub struct IkNode<J, S>
where
    J: JointTrajectoryClient,
    S: Speaker,
{
    joint_trajectory_client: J,
    speaker: S,
    mode: String,
    submode: String,
    step_duration: Duration,
    ik_solver_with_chain: Arc<IkSolverWithChain>,
    inner: IkNodeInner,
}

impl<J, S> IkNode<J, S>
where
    J: JointTrajectoryClient,
    S: Speaker,
{
    pub fn new(
        mode: String,
        joint_trajectory_client: J,
        move_step_linear: [f64; 3],
        move_step_angular: [f64; 3],
        step_duration: Duration,
        speaker: S,
        ik_solver_with_chain: Arc<IkSolverWithChain>,
    ) -> Self {
        Self {
            joint_trajectory_client,
            speaker,
            mode,
            submode: "".to_string(),
            step_duration,
            ik_solver_with_chain,
            inner: IkNodeInner::new(move_step_linear, move_step_angular),
        }
    }

    pub fn new_from_config(
        config: IkNodeConfig,
        joint_trajectory_client: J,
        speaker: S,
        ik_solver_with_chain: Arc<IkSolverWithChain>,
    ) -> Self {
        Self::new(
            config.mode,
            joint_trajectory_client,
            config.move_step_linear,
            config.move_step_angular,
            Duration::from_secs_f64(config.step_duration_secs),
            speaker,
            ik_solver_with_chain,
        )
    }
}

#[async_trait]
impl<N, S> ControlNode for IkNode<N, S>
where
    N: JointTrajectoryClient,
    S: Speaker,
{
    fn set_event(&mut self, event: GamepadEvent) {
        self.inner.set_event(event);
    }

    async fn proc(&self) {
        if self.inner.is_sending {
            let angular_velocity = self.inner.angular_velocity;
            let linear_velocity = self.inner.get_linear_velocity();
            let current_positions = self
                .joint_trajectory_client
                .current_joint_positions()
                .unwrap();
            self.ik_solver_with_chain
                .set_joint_positions_clamped(&current_positions);
            let current_pose = self.ik_solver_with_chain.end_transform();
            let rotated = current_pose
                * k::UnitQuaternion::from_euler_angles(
                    angular_velocity.x,
                    angular_velocity.y,
                    angular_velocity.z,
                );
            let target_pose = rotated * Translation3::from(linear_velocity);
            if self.ik_solver_with_chain.solve(&target_pose).is_ok() {
                let pos = self.ik_solver_with_chain.joint_positions();
                self.joint_trajectory_client
                    .send_joint_positions(pos, self.step_duration)
                    .unwrap()
                    .await
                    .unwrap();
            } else {
                self.speaker.speak("ik fail").unwrap().await.unwrap();
            }
        }
    }

    fn mode(&self) -> &str {
        &self.mode
    }

    fn submode(&self) -> &str {
        &self.submode
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct IkNodeConfig {
    pub mode: String,
    #[serde(default = "default_move_step_angular")]
    pub move_step_angular: [f64; 3],
    #[serde(default = "default_move_step_linear")]
    pub move_step_linear: [f64; 3],
    #[serde(default = "default_step_duration_secs")]
    pub step_duration_secs: f64,
}

const fn default_move_step_angular() -> [f64; 3] {
    [0.05, 0.05, 0.17]
}

const fn default_move_step_linear() -> [f64; 3] {
    [0.01, 0.01, 0.01]
}

const fn default_step_duration_secs() -> f64 {
    0.1
}

#[cfg(test)]
mod tests {
    use assert_approx_eq::*;

    use super::*;

    #[test]
    fn test_default_move_step_angular() {
        let def = default_move_step_angular();

        assert_eq!(def.len(), 3_usize);
        assert_approx_eq!(def[0], 0.05_f64);
        assert_approx_eq!(def[1], 0.05_f64);
        assert_approx_eq!(def[2], 0.17_f64);
    }

    #[test]
    fn test_default_move_step_linear() {
        let def = default_move_step_linear();

        assert_eq!(def.len(), 3_usize);
        assert_approx_eq!(def[0], 0.01_f64);
        assert_approx_eq!(def[1], 0.01_f64);
        assert_approx_eq!(def[2], 0.01_f64);
    }

    #[test]
    fn test_default_step_duration_secs() {
        let def = default_step_duration_secs();

        assert_approx_eq!(def, 0.1_f64);
    }
}
