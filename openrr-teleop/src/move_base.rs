use arci::{
    gamepad::{Axis, Button, GamepadEvent},
    BaseVelocity, MoveBase,
};
use async_trait::async_trait;
use parking_lot::Mutex;

use super::control_mode::ControlMode;

const BASE_LINEAR_VEL_AXIS_GAIN: f64 = 0.5;
const BASE_ANGULAR_VEL_AXIS_GAIN: f64 = 1.5;
const BASE_TURBO_GAIN: f64 = 2.0;

struct MoveBaseModeInner {
    vel: BaseVelocity,
    is_enabled: bool,
    is_turbo: bool,
}

impl MoveBaseModeInner {
    fn new() -> Self {
        Self {
            vel: BaseVelocity::default(),
            is_enabled: false,
            is_turbo: false,
        }
    }

    fn handle_event(&mut self, ev: GamepadEvent) -> bool {
        let mut should_stop = false;
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
                self.vel = BaseVelocity::default();
                should_stop = true;
            }
            GamepadEvent::ButtonPressed(Button::LeftTrigger2) => {
                self.is_turbo = true;
            }
            GamepadEvent::ButtonReleased(Button::LeftTrigger2) => {
                self.is_turbo = false;
            }
            GamepadEvent::Disconnected => {
                self.is_enabled = false;
                self.is_turbo = false;
                self.vel = BaseVelocity::default();
                should_stop = true;
            }
            _ => {}
        }
        should_stop
    }

    fn get_target_velocity(&self) -> Option<BaseVelocity> {
        if self.is_enabled {
            if self.is_turbo {
                let turbo_vel = self.vel * BASE_TURBO_GAIN;
                Some(turbo_vel)
            } else {
                Some(self.vel)
            }
        } else {
            None
        }
    }
}

pub struct MoveBaseMode<T: MoveBase> {
    move_base: T,
    mode: String,
    submode: String,
    inner: Mutex<MoveBaseModeInner>,
}

impl<T> MoveBaseMode<T>
where
    T: MoveBase,
{
    pub fn new(mode: String, move_base: T) -> Self {
        Self {
            move_base,
            mode,
            submode: "".to_string(),
            inner: Mutex::new(MoveBaseModeInner::new()),
        }
    }
}

#[async_trait]
impl<T> ControlMode for MoveBaseMode<T>
where
    T: MoveBase,
{
    fn handle_event(&self, ev: GamepadEvent) {
        if self.inner.lock().handle_event(ev) {
            // stop immediately
            self.move_base
                .send_velocity(&BaseVelocity::default())
                .unwrap();
        }
    }

    async fn proc(&self) {
        if let Some(v) = self.inner.lock().get_target_velocity() {
            self.move_base.send_velocity(&v).unwrap();
        }
    }

    fn mode(&self) -> &str {
        &self.mode
    }

    fn submode(&self) -> String {
        self.submode.to_owned()
    }
}

#[cfg(test)]
mod tests {
    use arci::DummyMoveBase;
    use assert_approx_eq::*;

    use super::*;

    #[test]
    fn test_move_mode_new() {
        let mode_name = String::from("tested");
        let base = DummyMoveBase::new();
        let mode = MoveBaseMode::new(mode_name.clone(), base);

        assert_eq!(
            format!("{:?}", mode.move_base),
            format!("{:?}", DummyMoveBase::new())
        );
        assert_eq!(mode.mode, mode_name);
        assert_eq!(mode.submode, String::from(""));
        assert_eq!(
            format!("{:?}", mode.inner.lock().vel),
            format!("{:?}", BaseVelocity::default())
        );
        assert!(!mode.inner.lock().is_enabled);
        assert!(!mode.inner.lock().is_turbo);
    }

    #[test]
    fn test_move_mode_get() {
        let mode_name = String::from("tested");
        let base = DummyMoveBase::new();
        let mode = MoveBaseMode::new(mode_name.clone(), base);

        let base_mode = mode.mode();
        assert_eq!(base_mode, mode_name);
        let base_sub_mode = mode.submode();
        assert_eq!(base_sub_mode, String::from(""));
    }

    #[tokio::test]
    async fn test_move_mode_proc() {
        let mode_name = String::from("tested");
        const X: f64 = 1.2;
        const Y: f64 = 3.5;
        const THETA: f64 = 1.8;
        let mode = MoveBaseMode {
            move_base: DummyMoveBase::new(),
            mode: mode_name.clone(),
            submode: "".to_string(),
            inner: Mutex::new(MoveBaseModeInner {
                vel: BaseVelocity {
                    x: X,
                    y: Y,
                    theta: THETA,
                },
                is_enabled: false,
                is_turbo: false,
            }),
        };
        mode.proc().await;
        let current = mode.move_base.current_velocity().unwrap();
        assert_approx_eq!(current.x, 0.0);
        assert_approx_eq!(current.y, 0.0);
        assert_approx_eq!(current.theta, 0.0);
        println!("{:?} {current:?}", mode.inner.lock().vel);

        let mode = MoveBaseMode {
            move_base: DummyMoveBase::new(),
            mode: mode_name.clone(),
            submode: "".to_string(),
            inner: Mutex::new(MoveBaseModeInner {
                vel: BaseVelocity {
                    x: 1.2,
                    y: 3.5,
                    theta: 1.8,
                },
                is_enabled: false,
                is_turbo: true,
            }),
        };
        mode.proc().await;
        let current = mode.move_base.current_velocity().unwrap();
        assert_approx_eq!(current.x, 0.0);
        assert_approx_eq!(current.y, 0.0);
        assert_approx_eq!(current.theta, 0.0);
        println!("{:?} {current:?}", mode.inner.lock().vel);

        let mode = MoveBaseMode {
            move_base: DummyMoveBase::new(),
            mode: mode_name.clone(),
            submode: "".to_string(),
            inner: Mutex::new(MoveBaseModeInner {
                vel: BaseVelocity {
                    x: 1.2,
                    y: 3.5,
                    theta: 1.8,
                },
                is_enabled: true,
                is_turbo: false,
            }),
        };
        mode.proc().await;
        let current = mode.move_base.current_velocity().unwrap();
        assert_approx_eq!(current.x, X);
        assert_approx_eq!(current.y, Y);
        assert_approx_eq!(current.theta, THETA);
        println!("{:?} {current:?}", mode.inner.lock().vel);

        let mode = MoveBaseMode {
            move_base: DummyMoveBase::new(),
            mode: mode_name.clone(),
            submode: "".to_string(),
            inner: Mutex::new(MoveBaseModeInner {
                vel: BaseVelocity {
                    x: 1.2_f64,
                    y: 3.5,
                    theta: 1.8,
                },
                is_enabled: true,
                is_turbo: true,
            }),
        };
        mode.proc().await;
        let current = mode.move_base.current_velocity().unwrap();
        assert_approx_eq!(current.x, X * 2.0);
        assert_approx_eq!(current.y, Y * 2.0);
        assert_approx_eq!(current.theta, THETA * 2.0);
        println!("{:?} {current:?}", mode.inner.lock().vel);
    }
}
