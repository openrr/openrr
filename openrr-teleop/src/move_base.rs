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

#[cfg(test)]
mod tests {
    use arci::DummyMoveBase;
    use assert_approx_eq::*;

    use super::*;

    #[test]
    fn test_move_node_new() {
        let mode = String::from("tested");
        let base = DummyMoveBase::new();
        let node = MoveBaseNode::new(mode.clone(), base);

        assert_eq!(
            format!("{:?}", node.move_base),
            format!("{:?}", DummyMoveBase::new())
        );
        assert_eq!(node.mode, mode);
        assert_eq!(node.submode, String::from(""));
        assert_eq!(
            format!("{:?}", node.vel),
            format!("{:?}", BaseVelocity::default())
        );
        assert!(!node.is_enabled);
        assert!(!node.is_turbo);
    }

    #[test]
    fn test_move_node_get() {
        let mode = String::from("tested");
        let base = DummyMoveBase::new();
        let node = MoveBaseNode::new(mode.clone(), base);

        let base_mode = node.mode();
        assert_eq!(base_mode, mode);
        let base_sub_mode = node.submode();
        assert_eq!(base_sub_mode, &String::from(""));
    }

    #[tokio::test]
    async fn test_move_node_proc() {
        let mode = String::from("tested");
        const X: f64 = 1.2;
        const Y: f64 = 3.5;
        const THETA: f64 = 1.8;
        let node = MoveBaseNode {
            move_base: DummyMoveBase::new(),
            mode: mode.clone(),
            submode: "".to_string(),
            vel: BaseVelocity {
                x: X,
                y: Y,
                theta: THETA,
            },
            is_enabled: false,
            is_turbo: false,
        };
        node.proc().await;
        let current = node.move_base.current_velocity().unwrap();
        assert_approx_eq!(current.x, 0.0);
        assert_approx_eq!(current.y, 0.0);
        assert_approx_eq!(current.theta, 0.0);
        println!("{:?} {:?}", node.vel, current);

        let node = MoveBaseNode {
            move_base: DummyMoveBase::new(),
            mode: mode.clone(),
            submode: "".to_string(),
            vel: BaseVelocity {
                x: 1.2,
                y: 3.5,
                theta: 1.8,
            },
            is_enabled: false,
            is_turbo: true,
        };
        node.proc().await;
        let current = node.move_base.current_velocity().unwrap();
        assert_approx_eq!(current.x, 0.0);
        assert_approx_eq!(current.y, 0.0);
        assert_approx_eq!(current.theta, 0.0);
        println!("{:?} {:?}", node.vel, current);

        let node = MoveBaseNode {
            move_base: DummyMoveBase::new(),
            mode: mode.clone(),
            submode: "".to_string(),
            vel: BaseVelocity {
                x: 1.2,
                y: 3.5,
                theta: 1.8,
            },
            is_enabled: true,
            is_turbo: false,
        };
        node.proc().await;
        let current = node.move_base.current_velocity().unwrap();
        assert_approx_eq!(current.x, X);
        assert_approx_eq!(current.y, Y);
        assert_approx_eq!(current.theta, THETA);
        println!("{:?} {:?}", node.vel, current);

        let node = MoveBaseNode {
            move_base: DummyMoveBase::new(),
            mode: mode.clone(),
            submode: "".to_string(),
            vel: BaseVelocity {
                x: 1.2_f64,
                y: 3.5,
                theta: 1.8,
            },
            is_enabled: true,
            is_turbo: true,
        };
        node.proc().await;
        let current = node.move_base.current_velocity().unwrap();
        assert_approx_eq!(current.x, X * 2.0);
        assert_approx_eq!(current.y, Y * 2.0);
        assert_approx_eq!(current.theta, THETA * 2.0);
        println!("{:?} {:?}", node.vel, current);
    }
}
