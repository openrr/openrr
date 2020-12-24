use super::control_node::ControlNode;
use arci::gamepad::{Axis, Button, GamepadEvent};
use arci::{JointTrajectoryClient, Speaker};
use async_trait::async_trait;
use k::{Translation3, Vector3};
use openrr_client::IkSolverWithChain;
use std::time::Duration;

const AXIS_GAIN: f64 = 0.2;
const IK_POSITION_TURBO_GAIN: f64 = 2.0;

pub struct IkNode<J, S>
where
    J: JointTrajectoryClient,
    S: Speaker,
{
    joint_trajectory_client: J,
    speaker: S,
    mode: String,
    submode: String,
    linear_velocity: Vector3<f64>,
    angular_velocity: Vector3<f64>,
    move_step_linear: f64,
    move_step_angular: f64,
    step_duration: Duration,
    ik_solver_with_chain: IkSolverWithChain,
    is_turbo: bool,
    is_sending: bool,
    pub constraints: k::Constraints,
}

impl<J, S> IkNode<J, S>
where
    J: JointTrajectoryClient,
    S: Speaker,
{
    pub fn new(
        mode: String,
        joint_trajectory_client: J,
        move_step_linear: f64,
        move_step_angular: f64,
        step_duration: Duration,
        speaker: S,
        ik_solver_with_chain: IkSolverWithChain,
    ) -> Self {
        Self {
            joint_trajectory_client,
            speaker,
            mode,
            submode: "".to_string(),
            linear_velocity: Vector3::new(0.0, 0.0, 0.0),
            angular_velocity: Vector3::new(0.0, 0.0, 0.0),
            move_step_linear,
            move_step_angular,
            step_duration,
            ik_solver_with_chain,
            is_turbo: false,
            is_sending: false,
            constraints: k::Constraints::default(),
        }
    }
}

impl<N, S> IkNode<N, S>
where
    N: JointTrajectoryClient,
    S: Speaker,
{
    fn clear_velocity(&mut self) {
        self.linear_velocity.x = 0.0;
        self.linear_velocity.y = 0.0;
        self.linear_velocity.z = 0.0;
        self.angular_velocity.x = 0.0;
        self.angular_velocity.y = 0.0;
        self.angular_velocity.z = 0.0;
    }
}

#[async_trait]
impl<N, S> ControlNode for IkNode<N, S>
where
    N: JointTrajectoryClient,
    S: Speaker,
{
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
                self.linear_velocity.z = -self.move_step_linear;
            }
            GamepadEvent::ButtonReleased(Button::South) => {
                self.linear_velocity.z = 0.0;
            }
            GamepadEvent::ButtonPressed(Button::West) => {
                self.linear_velocity.z = self.move_step_linear;
            }
            GamepadEvent::ButtonReleased(Button::West) => {
                self.linear_velocity.z = 0.0;
            }
            GamepadEvent::AxisChanged(Axis::RightStickY, v) => {
                self.linear_velocity.x = self.move_step_linear * v * AXIS_GAIN;
            }
            GamepadEvent::AxisChanged(Axis::RightStickX, v) => {
                self.linear_velocity.y = self.move_step_linear * v * AXIS_GAIN;
            }
            GamepadEvent::AxisChanged(Axis::LeftStickX, v) => {
                self.angular_velocity.z = self.move_step_angular * v * AXIS_GAIN;
            }
            GamepadEvent::AxisChanged(Axis::LeftStickY, v) => {
                self.angular_velocity.x = self.move_step_angular * v * AXIS_GAIN;
            }
            _ => {}
        }
    }
    async fn proc(&self) {
        if self.is_sending {
            let current_positions = self
                .joint_trajectory_client
                .current_joint_positions()
                .unwrap();
            self.ik_solver_with_chain
                .set_joint_positions_clamped(&current_positions);
            let current_pose = self.ik_solver_with_chain.end_transform();
            let rotated = current_pose
                * k::UnitQuaternion::from_euler_angles(
                    self.angular_velocity.x,
                    self.angular_velocity.y,
                    self.angular_velocity.z,
                );
            let target_pose = rotated
                * Translation3::from(
                    self.linear_velocity
                        * if self.is_turbo {
                            IK_POSITION_TURBO_GAIN
                        } else {
                            1.0
                        },
                );
            if self
                .ik_solver_with_chain
                .solve_with_constraints(&target_pose, &self.constraints)
                .is_ok()
            {
                let pos = self.ik_solver_with_chain.joint_positions();
                self.joint_trajectory_client
                    .send_joint_positions(pos, self.step_duration)
                    .await
                    .unwrap();
            } else {
                self.speaker.speak("ik fail");
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
