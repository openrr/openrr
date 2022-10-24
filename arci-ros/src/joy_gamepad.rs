use std::{collections::HashMap, sync::Arc};

use arci::{gamepad::*, *};
use parking_lot::Mutex;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::msg::sensor_msgs::Joy;

pub struct JoyGamepad {
    _last_joy_msg: Arc<Mutex<Joy>>,
    rx: flume::Receiver<GamepadEvent>,
    _sub: rosrust::Subscriber,
}

impl JoyGamepad {
    pub fn new(
        topic_name: &str,
        button_mapping: HashMap<usize, arci::gamepad::Button>,
        axis_mapping: HashMap<usize, arci::gamepad::Axis>,
    ) -> Self {
        const DEAD_ZONE: f32 = 0.00001;
        const TOPIC_BUFFER_SIZE: usize = 100;
        let last_joy_msg = Arc::new(Mutex::new(Joy::default()));
        let (tx, rx) = flume::unbounded();
        let tx_for_stop = tx.clone();
        // spawn for stop by Ctrl-C
        tokio::spawn(async move {
            tokio::signal::ctrl_c().await.unwrap();
            tx_for_stop.send(GamepadEvent::Unknown).unwrap();
        });

        let last_joy_msg_clone = last_joy_msg.clone();
        let _sub = rosrust::subscribe(topic_name, TOPIC_BUFFER_SIZE, move |joy_msg: Joy| {
            let mut last_joy = last_joy_msg_clone.lock();
            // initialize last_joy_msg
            if last_joy.buttons.len() < joy_msg.buttons.len() {
                last_joy.buttons.resize(joy_msg.buttons.len(), 0);
            }
            if last_joy.axes.len() < joy_msg.axes.len() {
                last_joy.axes.resize(joy_msg.axes.len(), 0.0);
            }
            for (idx_ref, button) in button_mapping.iter() {
                let idx = *idx_ref;
                if joy_msg.buttons.len() <= idx {
                    rosrust::ros_err!(
                        "buttons index is out of range, ignored: input={idx}, size={}",
                        joy_msg.buttons.len()
                    );
                } else if last_joy.buttons[idx] == 0 && joy_msg.buttons[idx] == 1 {
                    tx.send(GamepadEvent::ButtonPressed(button.to_owned()))
                        .unwrap();
                } else if last_joy.buttons[idx] == 1 && joy_msg.buttons[idx] == 0 {
                    tx.send(GamepadEvent::ButtonReleased(button.to_owned()))
                        .unwrap();
                }
            }
            for (idx_ref, axis) in axis_mapping.iter() {
                let idx = *idx_ref;
                if joy_msg.axes.len() <= idx {
                    rosrust::ros_err!(
                        "axes index is out of range, ignored: input={idx}, size={}",
                        joy_msg.buttons.len()
                    );
                } else if (last_joy.axes[idx] - joy_msg.axes[idx]).abs() > DEAD_ZONE {
                    tx.send(GamepadEvent::AxisChanged(
                        axis.to_owned(),
                        joy_msg.axes[idx] as f64,
                    ))
                    .unwrap();
                }
            }
            *last_joy = joy_msg;
        })
        .unwrap();
        Self {
            _last_joy_msg: last_joy_msg,
            rx,
            _sub,
        }
    }

    pub fn new_from_config(config: &JoyGamepadConfig) -> Self {
        let topic_name = &config.topic_name;
        let mut button_map = HashMap::new();
        for (key, &value) in config.button_map.iter() {
            button_map.insert(key.parse::<usize>().unwrap(), value);
        }
        let mut axis_map = HashMap::new();
        for (key, &value) in config.axis_map.iter() {
            axis_map.insert(key.parse::<usize>().unwrap(), value);
        }
        Self::new(topic_name, button_map, axis_map)
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct JoyGamepadConfig {
    pub topic_name: String,
    pub button_map: HashMap<String, Button>,
    pub axis_map: HashMap<String, Axis>,
}

impl JoyGamepadConfig {
    pub fn new() -> Self {
        Self {
            topic_name: default_topic_name(),
            button_map: default_button_map(),
            axis_map: default_axis_map(),
        }
    }
}

impl Default for JoyGamepadConfig {
    fn default() -> Self {
        JoyGamepadConfig::new()
    }
}

fn default_topic_name() -> String {
    "joy".to_string()
}

fn default_button_map() -> HashMap<String, Button> {
    let mut button_map = HashMap::new();
    button_map.insert("0".to_string(), arci::gamepad::Button::South);
    button_map.insert("1".to_string(), arci::gamepad::Button::East);
    button_map.insert("2".to_string(), arci::gamepad::Button::West);
    button_map.insert("3".to_string(), arci::gamepad::Button::North);
    button_map.insert("4".to_string(), arci::gamepad::Button::LeftTrigger2);
    button_map.insert("5".to_string(), arci::gamepad::Button::RightTrigger2);
    button_map
}

fn default_axis_map() -> HashMap<String, Axis> {
    let mut axis_map = HashMap::new();
    axis_map.insert("0".to_string(), arci::gamepad::Axis::LeftStickX);
    axis_map.insert("1".to_string(), arci::gamepad::Axis::LeftStickY);
    axis_map.insert("2".to_string(), arci::gamepad::Axis::LeftTrigger);
    axis_map.insert("3".to_string(), arci::gamepad::Axis::RightStickX);
    axis_map.insert("4".to_string(), arci::gamepad::Axis::RightStickY);
    axis_map.insert("5".to_string(), arci::gamepad::Axis::RightTrigger);
    axis_map.insert("6".to_string(), arci::gamepad::Axis::DPadX);
    axis_map.insert("7".to_string(), arci::gamepad::Axis::DPadY);
    axis_map
}

#[async_trait]
impl Gamepad for JoyGamepad {
    async fn next_event(&self) -> GamepadEvent {
        if let Ok(ev) = self.rx.recv_async().await {
            ev
        } else {
            GamepadEvent::Unknown
        }
    }

    fn stop(&self) {}
}
