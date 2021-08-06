use std::{future, sync::Mutex};

use async_trait::async_trait;

use crate::gamepad::{Axis, Button, Gamepad, GamepadEvent};

/// Dummy Gamepad for debug or tests.
#[derive(Debug)]
pub struct DummyGamepad {
    pub events: Vec<GamepadEvent>,
    index: Mutex<usize>,
    stopped: Mutex<bool>,
}

impl DummyGamepad {
    /// Creates a new `DummyGamepad` which returns the given events.
    pub fn new(events: Vec<GamepadEvent>) -> Self {
        Self {
            events,
            index: Mutex::default(),
            stopped: Mutex::default(),
        }
    }

    /// Creates a new `DummyGamepad` which returns all patterns of the GamepadEvent.
    pub fn with_all_events() -> Self {
        const ALL_BUTTONS: &[Button] = &[
            Button::South,
            Button::East,
            Button::North,
            Button::West,
            Button::LeftTrigger,
            Button::LeftTrigger2,
            Button::RightTrigger,
            Button::RightTrigger2,
            Button::Select,
            Button::Start,
            Button::Mode,
            Button::LeftThumb,
            Button::RightThumb,
            Button::DPadUp,
            Button::DPadDown,
            Button::DPadLeft,
            Button::DPadRight,
            Button::Unknown,
        ];
        const ALL_AXIS: &[Axis] = &[
            Axis::LeftStickX,
            Axis::LeftStickY,
            Axis::LeftTrigger,
            Axis::RightStickX,
            Axis::RightStickY,
            Axis::RightTrigger,
            Axis::DPadX,
            Axis::DPadY,
            Axis::Unknown,
        ];

        let mut events = vec![];
        for &b in ALL_BUTTONS {
            events.push(GamepadEvent::ButtonPressed(b));
        }
        for &b in ALL_BUTTONS {
            events.push(GamepadEvent::ButtonReleased(b));
        }
        for &a in ALL_AXIS {
            events.push(GamepadEvent::AxisChanged(a, 1.0));
            events.push(GamepadEvent::AxisChanged(a, 0.0));
            events.push(GamepadEvent::AxisChanged(a, -1.0));
        }
        events.push(GamepadEvent::Unknown);
        Self::new(events)
    }

    pub fn is_stopped(&self) -> bool {
        *self.stopped.lock().unwrap()
    }
}

#[async_trait]
impl Gamepad for DummyGamepad {
    async fn next_event(&self) -> GamepadEvent {
        *self.stopped.lock().unwrap() = false;
        {
            let mut index = self.index.lock().unwrap();
            if let Some(event) = self.events.get(*index).cloned() {
                *index += 1;
                return event;
            }
        }
        future::pending::<()>().await;
        unreachable!()
    }

    fn stop(&self) {
        *self.stopped.lock().unwrap() = true;
    }
}
