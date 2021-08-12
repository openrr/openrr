//! [`arci::Gamepad`] implementation using [gilrs](https://gitlab.com/gilrs-project/gilrs).

#![warn(rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

use std::{
    collections::HashMap,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

use arci::{gamepad::*, *};
use schemars::{gen::SchemaGenerator, schema::Schema, JsonSchema};
use serde::{Deserialize, Serialize};
use serde_with::serde_as;
#[cfg(not(target_os = "macos"))]
use tracing::info;
use tracing::{debug, error};

#[serde_as]
#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct Map {
    #[serde_as(as = "Vec<(_, _)>")]
    #[serde(default = "default_button_map")]
    button_map: HashMap<gilrs::Button, Button>,
    #[serde_as(as = "Vec<(_, _)>")]
    #[serde(default = "default_axis_map")]
    axis_map: HashMap<gilrs::Axis, Axis>,
    #[serde_as(as = "Vec<(_, _)>")]
    #[serde(default = "default_axis_value_map")]
    axis_value_map: HashMap<Axis, f64>,
}

impl Map {
    pub fn new() -> Self {
        Self {
            button_map: default_button_map(),
            axis_map: default_axis_map(),
            axis_value_map: default_axis_value_map(),
        }
    }

    pub fn new_playstation() -> Self {
        let mut button_map = default_button_map();
        button_map.insert(gilrs::Button::East, Button::South);
        button_map.insert(gilrs::Button::C, Button::East);
        button_map.insert(gilrs::Button::North, Button::North);
        button_map.insert(gilrs::Button::South, Button::West);
        button_map.insert(gilrs::Button::West, Button::LeftTrigger);
        button_map.insert(gilrs::Button::LeftTrigger, Button::LeftTrigger2);
        button_map.insert(gilrs::Button::Z, Button::RightTrigger);
        button_map.insert(gilrs::Button::RightTrigger, Button::RightTrigger2);
        button_map.insert(gilrs::Button::LeftTrigger2, Button::Select);
        button_map.insert(gilrs::Button::RightTrigger2, Button::Start);
        button_map.insert(gilrs::Button::Select, Button::LeftThumb);
        button_map.insert(gilrs::Button::Start, Button::RightThumb);

        let mut axis_map = default_axis_map();
        axis_map.insert(gilrs::Axis::LeftZ, Axis::RightStickX);
        axis_map.insert(gilrs::Axis::RightZ, Axis::RightStickY);

        let mut axis_value_map = default_axis_value_map();
        axis_value_map.insert(Axis::RightStickX, -1.0);
        axis_value_map.insert(Axis::RightStickY, -1.0);

        Self {
            button_map,
            axis_map,
            axis_value_map,
        }
    }

    fn convert_button(&self, b: gilrs::Button) -> Button {
        if let Some(e) = self.button_map.get(&b) {
            debug!("convert_button {:?} -> {:?}", b, e);
            *e
        } else {
            debug!("unknown map {:?}", b);
            Button::Unknown
        }
    }

    fn convert_axis(&self, a: gilrs::Axis, v: f32) -> (Axis, f64) {
        if let Some(e) = self.axis_map.get(&a) {
            debug!("convert_axis {:?} -> {:?}", a, e);
            (*e, v as f64 * self.axis_value_map.get(e).unwrap_or(&1.0))
        } else {
            debug!("unknown map {:?}", a);
            (Axis::Unknown, 0.0)
        }
    }

    fn convert_event(&self, e: gilrs::EventType) -> Option<GamepadEvent> {
        match e {
            gilrs::EventType::ButtonPressed(b, _c) => {
                Some(GamepadEvent::ButtonPressed(self.convert_button(b)))
            }
            gilrs::EventType::ButtonReleased(b, _c) => {
                Some(GamepadEvent::ButtonReleased(self.convert_button(b)))
            }
            gilrs::EventType::AxisChanged(a, v, _c) => {
                let (axis, value) = self.convert_axis(a, v);
                Some(GamepadEvent::AxisChanged(axis, value))
            }
            _ => None,
        }
    }
}

impl Default for Map {
    fn default() -> Self {
        Self::new()
    }
}

impl JsonSchema for Map {
    fn schema_name() -> String {
        "Map".to_string()
    }

    fn json_schema(gen: &mut SchemaGenerator) -> Schema {
        // https://docs.rs/gilrs/0.8/gilrs/ev/enum.Button.html
        #[allow(dead_code)]
        #[derive(JsonSchema)]
        enum GilrsButton {
            South,
            East,
            North,
            West,
            C,
            Z,
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
        // https://docs.rs/gilrs/0.8/gilrs/ev/enum.Axis.html
        #[allow(dead_code)]
        #[derive(JsonSchema)]
        enum GilrsAxis {
            LeftStickX,
            LeftStickY,
            LeftZ,
            RightStickX,
            RightStickY,
            RightZ,
            DPadX,
            DPadY,
            Unknown,
        }
        #[allow(dead_code)]
        #[derive(JsonSchema)]
        struct MapRepr {
            button_map: Vec<(GilrsButton, Button)>,
            axis_map: Vec<(GilrsAxis, Axis)>,
            axis_value_map: Vec<(Axis, f64)>,
        }

        MapRepr::json_schema(gen)
    }
}

fn default_button_map() -> HashMap<gilrs::Button, Button> {
    let mut button_map = HashMap::new();
    button_map.insert(gilrs::Button::South, Button::South);
    button_map.insert(gilrs::Button::East, Button::East);
    button_map.insert(gilrs::Button::North, Button::North);
    button_map.insert(gilrs::Button::West, Button::West);
    button_map.insert(gilrs::Button::LeftTrigger, Button::LeftTrigger);
    button_map.insert(gilrs::Button::LeftTrigger2, Button::LeftTrigger2);
    button_map.insert(gilrs::Button::RightTrigger, Button::RightTrigger);
    button_map.insert(gilrs::Button::RightTrigger2, Button::RightTrigger2);
    button_map.insert(gilrs::Button::Select, Button::Select);
    button_map.insert(gilrs::Button::Start, Button::Start);
    button_map.insert(gilrs::Button::Mode, Button::Mode);
    button_map.insert(gilrs::Button::LeftThumb, Button::LeftThumb);
    button_map.insert(gilrs::Button::RightThumb, Button::RightThumb);
    button_map.insert(gilrs::Button::DPadUp, Button::DPadUp);
    button_map.insert(gilrs::Button::DPadDown, Button::DPadDown);
    button_map.insert(gilrs::Button::DPadLeft, Button::DPadLeft);
    button_map.insert(gilrs::Button::DPadRight, Button::DPadRight);
    button_map
}

fn default_axis_map() -> HashMap<gilrs::Axis, Axis> {
    let mut axis_map = HashMap::new();
    axis_map.insert(gilrs::Axis::LeftStickX, Axis::LeftStickX);
    axis_map.insert(gilrs::Axis::LeftStickY, Axis::LeftStickY);
    axis_map.insert(gilrs::Axis::RightStickX, Axis::RightStickX);
    axis_map.insert(gilrs::Axis::RightStickY, Axis::RightStickY);
    axis_map.insert(gilrs::Axis::DPadX, Axis::DPadX);
    axis_map.insert(gilrs::Axis::DPadY, Axis::DPadY);
    axis_map
}

fn default_axis_value_map() -> HashMap<Axis, f64> {
    let mut axis_value_map = HashMap::new();
    axis_value_map.insert(Axis::RightStickX, -1.0);
    axis_value_map.insert(Axis::LeftStickX, -1.0);
    axis_value_map
}

#[derive(Debug)]
pub struct GilGamepad {
    rx: flume::Receiver<GamepadEvent>,
    is_running: Arc<AtomicBool>,
}

impl GilGamepad {
    pub fn new(id: usize, map: Map) -> Self {
        let (tx, rx) = flume::unbounded();
        let is_running = Arc::new(AtomicBool::new(true));
        let is_running_cloned = is_running.clone();
        std::thread::spawn(move || {
            let mut gil = gilrs::Gilrs::new().unwrap();
            let mut selected_gamepad_id = None;
            // TODO: On MacOS `gamepads()` does not works.
            #[cfg(not(target_os = "macos"))]
            {
                let mut is_found = false;
                for (connected_id, gamepad) in gil.gamepads() {
                    info!("{} is {:?}", gamepad.name(), gamepad.power_info());
                    if id == Into::<usize>::into(connected_id) {
                        is_found = true;
                        selected_gamepad_id = Some(connected_id);
                    }
                }
                if !is_found {
                    panic!("No Gamepad id={} is found", id);
                }
            }
            while is_running_cloned.load(Ordering::Relaxed) {
                if let Some(gamepad_id) = selected_gamepad_id {
                    let gamepad = gil.gamepad(gamepad_id);
                    if !gamepad.is_connected() {
                        error!("gamepad [{}] is disconnected", gamepad.name());
                        is_running_cloned.store(false, Ordering::Relaxed);
                        break;
                    }
                }
                // gil.next_event is no block. We have to polling it.
                match gil.next_event() {
                    Some(gilrs::Event {
                        id: recv_id, event, ..
                    }) => {
                        if id == Into::<usize>::into(recv_id) {
                            if let Some(e) = map.convert_event(event) {
                                tx.send(e).unwrap();
                            }
                        }
                    }
                    None => {
                        std::thread::sleep(Duration::from_secs_f64(0.01));
                    }
                }
            }
        });

        Self { rx, is_running }
    }

    pub fn new_from_config(config: GilGamepadConfig) -> Self {
        Self::new(config.device_id, config.map)
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, Default, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct GilGamepadConfig {
    #[serde(default)]
    device_id: usize,
    #[serde(default)]
    map: Map,
}

#[async_trait]
impl Gamepad for GilGamepad {
    async fn next_event(&self) -> GamepadEvent {
        match self.rx.recv_async().await {
            Ok(e) => e,
            Err(e) => {
                error!("recv error: {}", e);
                GamepadEvent::Unknown
            }
        }
    }

    fn stop(&self) {
        self.is_running.store(false, Ordering::Relaxed);
    }
}

impl Drop for GilGamepad {
    fn drop(&mut self) {
        self.stop();
    }
}

#[cfg(test)]
mod test {
    use std::collections::HashMap;

    use assert_approx_eq::assert_approx_eq;

    use super::*;

    #[test]
    fn test_playstation_map() {
        let m = Map::new_playstation();
        assert_eq!(
            m.convert_button(gilrs::Button::North),
            arci::gamepad::Button::North
        );
        assert_eq!(
            m.convert_button(gilrs::Button::South),
            arci::gamepad::Button::West
        );
        let (axis, value) = m.convert_axis(gilrs::Axis::RightStickX, 0.2);
        assert_eq!(axis, arci::gamepad::Axis::RightStickX);
        assert_approx_eq!(value, -0.2);
        let (a, v) = m.convert_axis(gilrs::Axis::RightZ, 0.1);
        assert_eq!(a, arci::gamepad::Axis::RightStickY);
        assert!((v - -0.1).abs() < 0.00001);
    }

    #[test]
    fn test_default_map() {
        let m = Map::default();
        assert_eq!(
            m.convert_button(gilrs::Button::North),
            arci::gamepad::Button::North
        );
        assert_eq!(
            m.convert_button(gilrs::Button::South),
            arci::gamepad::Button::South
        );
        let (a, v) = m.convert_axis(gilrs::Axis::RightStickY, 0.1);
        assert_eq!(a, arci::gamepad::Axis::RightStickY);
        assert!((v - 0.1).abs() < 0.00001);
    }

    #[test]
    fn test_make_map() {
        let m = Map {
            button_map: HashMap::new(),
            axis_map: HashMap::new(),
            axis_value_map: HashMap::new(),
        };
        assert_eq!(
            m.convert_button(gilrs::Button::North),
            arci::gamepad::Button::Unknown,
        );
        let (a, v) = m.convert_axis(gilrs::Axis::RightStickY, 0.1);
        assert_eq!(a, arci::gamepad::Axis::Unknown);
        assert!((v - 0.0).abs() < 0.00001);
    }
}
