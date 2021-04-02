use auto_impl::auto_impl;
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Button {
    South,
    East,
    North,
    West,
    LeftTrigger,
    LeftTrigger2,
    RightTrigger,
    RightTrigger2,
    Select,
    Start,
    Mode,
    LeftThumb,
    RightThumb,
    DPadUp,
    DPadDown,
    DPadLeft,
    DPadRight,
    Unknown,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Axis {
    LeftStickX,
    LeftStickY,
    LeftTrigger,
    RightStickX,
    RightStickY,
    RightTrigger,
    DPadX,
    DPadY,
    Unknown,
}

#[derive(Debug, Clone)]
pub enum GamepadEvent {
    ButtonPressed(Button),
    ButtonReleased(Button),
    AxisChanged(Axis, f64),
    Unknown,
}

#[auto_impl(Box, Arc)]
pub trait Gamepad: Send + Sync {
    fn next_event(&self) -> GamepadEvent;
    fn stop(&self);
}
