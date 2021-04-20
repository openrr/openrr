use arci::{
    gamepad::{Axis, Button, GamepadEvent},
    BaseVelocity, MoveBase,
};
use async_trait::async_trait;

use super::control_node::ControlNode;

const BASE_LINEAR_VEL_AXIS_GAIN: f64 = 0.5;
const BASE_ANGULAR_VEL_AXIS_GAIN: f64 = 1.5;
const BASE_TURBO_GAIN: f64 = 2.0;

pub struct MoveBaseNode<T: MoveBase> {
    move_base: T,
    mode: String,
    submode: String,
    vel: BaseVelocity,
    is_enabled: bool,
    is_turbo: bool,
}

impl<T> MoveBaseNode<T>
where
    T: MoveBase,
{
    pub fn new(mode: String, move_base: T) -> Self {
        Self {
            move_base,
            mode,
            submode: "".to_string(),
            vel: BaseVelocity::default(),
            is_enabled: false,
            is_turbo: false,
        }
    }
}

#[async_trait]
impl<T> ControlNode for MoveBaseNode<T>
where
    T: MoveBase,
{
    fn set_event(&mut self, ev: GamepadEvent) {
        match ev {
            GamepadEvent::AxisChanged(Axis::LeftStickX, v) => {
                self.vel.y = v * BASE_LINEAR_VEL_AXIS_GAIN
            }
            GamepadEvent::AxisChanged(Axis::LeftStickY, v) => {
                self.vel.x = v * BASE_LINEAR_VEL_AXIS_GAIN
            }
            GamepadEvent::AxisChanged(Axis::RightStickX, v) => {
                self.vel.theta = v * BASE_ANGULAR_VEL_AXIS_GAIN
            }
            GamepadEvent::ButtonPressed(Button::RightTrigger2) => {
                self.is_enabled = true;
            }
            GamepadEvent::ButtonReleased(Button::RightTrigger2) => {
                self.is_enabled = false;
                // stop immediately
                self.move_base
                    .send_velocity(&BaseVelocity::default())
                    .unwrap();
            }
            GamepadEvent::ButtonPressed(Button::LeftTrigger2) => {
                self.is_turbo = true;
            }
            GamepadEvent::ButtonReleased(Button::LeftTrigger2) => {
                self.is_turbo = false;
            }
            _ => {}
        }
    }

    async fn proc(&self) {
        if self.is_enabled {
            if self.is_turbo {
                let turbo_vel = self.vel * BASE_TURBO_GAIN;
                self.move_base.send_velocity(&turbo_vel).unwrap();
            } else {
                self.move_base.send_velocity(&self.vel).unwrap();
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
