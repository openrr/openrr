use crate::msg::sensor_msgs::Joy;
use arci::gamepad::*;
use arci::*;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

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
            let mut last_joy = last_joy_msg_clone.lock().unwrap();
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
                        "buttons index is out of range, ignored: input={}, size={}",
                        idx,
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
                        "axes index is out of range, ignored: input={}, size={}",
                        idx,
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
