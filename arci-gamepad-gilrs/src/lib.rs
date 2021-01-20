use arci::gamepad::*;
use arci::*;
use log::{debug, error, info};
use serde::{Deserialize, Serialize};
use serde_with::serde_as;
use std::{collections::HashMap, time::Duration};

#[serde_as]
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Map {
    #[serde_as(as = "Vec<(_, _)>")]
    button_map: HashMap<gilrs::Button, Button>,
    #[serde_as(as = "Vec<(_, _)>")]
    axis_map: HashMap<gilrs::Axis, Axis>,
    #[serde_as(as = "Vec<(_, _)>")]
    axis_value_map: HashMap<Axis, f64>,
}

impl Map {
    pub fn new() -> Self {
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

        let mut axis_map = HashMap::new();
        axis_map.insert(gilrs::Axis::LeftStickX, Axis::LeftStickX);
        axis_map.insert(gilrs::Axis::LeftStickY, Axis::LeftStickY);
        axis_map.insert(gilrs::Axis::RightStickX, Axis::RightStickX);
        axis_map.insert(gilrs::Axis::RightStickY, Axis::RightStickY);
        axis_map.insert(gilrs::Axis::DPadX, Axis::DPadX);
        axis_map.insert(gilrs::Axis::DPadY, Axis::DPadY);

        Self {
            button_map,
            axis_map,
            axis_value_map: HashMap::new(),
        }
    }

    pub fn new_playstation() -> Self {
        let mut button_map = HashMap::new();
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
        button_map.insert(gilrs::Button::Mode, Button::Mode);
        button_map.insert(gilrs::Button::Select, Button::LeftThumb);
        button_map.insert(gilrs::Button::Start, Button::RightThumb);
        button_map.insert(gilrs::Button::DPadUp, Button::DPadUp);
        button_map.insert(gilrs::Button::DPadDown, Button::DPadDown);
        button_map.insert(gilrs::Button::DPadLeft, Button::DPadLeft);
        button_map.insert(gilrs::Button::DPadRight, Button::DPadRight);

        let mut axis_map = HashMap::new();
        axis_map.insert(gilrs::Axis::LeftStickX, Axis::LeftStickX);
        axis_map.insert(gilrs::Axis::LeftStickY, Axis::LeftStickY);
        axis_map.insert(gilrs::Axis::LeftZ, Axis::RightStickX);
        axis_map.insert(gilrs::Axis::RightZ, Axis::RightStickY);
        axis_map.insert(gilrs::Axis::DPadX, Axis::DPadX);
        axis_map.insert(gilrs::Axis::DPadY, Axis::DPadY);
        let mut axis_value_map = HashMap::new();
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
            debug!("unkown map {:?}", b);
            Button::Unknown
        }
    }

    fn convert_axis(&self, a: gilrs::Axis, v: f32) -> (Axis, f64) {
        if let Some(e) = self.axis_map.get(&a) {
            debug!("convert_axis {:?} -> {:?}", a, e);
            (*e, v as f64 * self.axis_value_map.get(e).unwrap_or(&1.0))
        } else {
            debug!("unkown map {:?}", a);
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

#[cfg(test)]
mod test {
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

pub struct GilGamepad {
    rx: crossbeam_channel::Receiver<GamepadEvent>,
    _handle: std::thread::JoinHandle<()>,
}

impl GilGamepad {
    pub fn new(id: usize, map: Map) -> Self {
        let (tx, rx) = crossbeam_channel::unbounded();
        let _handle = std::thread::spawn(move || {
            let mut is_found = false;
            let mut gil = gilrs::Gilrs::new().unwrap();

            for (connected_id, gamepad) in gil.gamepads() {
                info!("{} is {:?}", gamepad.name(), gamepad.power_info());
                if id == connected_id.into() {
                    is_found = true;
                }
            }
            if !is_found {
                panic!("No Gamepad id={} is found", id);
            }
            loop {
                // gil.next_event is no block. We have to polling it.
                match gil.next_event() {
                    Some(gilrs::Event {
                        id: recv_id, event, ..
                    }) => {
                        if id == recv_id.into() {
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

        Self { rx, _handle }
    }
    pub fn new_from_config(config: GilGamepadConfig) -> Self {
        Self::new(config.device_id, config.map)
    }
}

#[async_trait]
impl Gamepad for GilGamepad {
    async fn next_event(&self) -> GamepadEvent {
        match self.rx.recv() {
            Ok(e) => e,
            Err(e) => {
                error!("recv error: {:?}", e);
                GamepadEvent::Unknown
            }
        }
    }
}

/* We need gamepad to test actually
#[cfg(test)]
mod test {
    use super::*;
    #[tokio::test]
    async fn test_gil() {
        let g = GilGamepad::new(0, Map::new_playstation());
        for _i in 0..100 {
            println!("Result = {:?}", g.next_event().await);
        }
    }
}
*/

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct GilGamepadConfig {
    device_id: usize,
    map: Map,
}
