use async_trait::async_trait;
use auto_impl::auto_impl;

#[derive(Debug, Copy, Clone, PartialEq)]
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

#[derive(Debug, Copy, Clone, PartialEq)]
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

#[async_trait]
#[auto_impl(Box, Rc, Arc)]
pub trait Gamepad {
    async fn next_event(&self) -> GamepadEvent;
}
