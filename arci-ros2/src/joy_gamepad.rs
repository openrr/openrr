use std::{collections::HashMap, sync::Arc};

use arci::{gamepad::*, *};
use futures::{Stream, StreamExt};
use parking_lot::Mutex;
use r2r::{sensor_msgs::msg::Joy, QosProfile};

/// TODO
pub struct Ros2JoyGamepad {
    last_joy_msg: Arc<Mutex<Joy>>,
    rx: flume::Receiver<GamepadEvent>,
    _node: Arc<Mutex<r2r::Node>>,
}

impl Ros2JoyGamepad {
    /// TODO
    pub fn new(
        ctx: r2r::Context,
        topic_name: &str,
        button_mapping: HashMap<usize, arci::gamepad::Button>,
        axis_mapping: HashMap<usize, arci::gamepad::Axis>,
    ) -> Self {
        let last_joy_msg = Arc::new(Mutex::new(Joy::default()));

        let (tx, rx) = flume::unbounded();
        let tx_for_stop = tx.clone();
        // spawn for stop by Ctrl-C
        tokio::spawn(async move {
            tokio::signal::ctrl_c().await.unwrap();
            tx_for_stop.send(GamepadEvent::Unknown).unwrap();
        });

        let mut node = r2r::Node::create(ctx, "openrr_ros2_joy_gamepad_node", "arci_ros2").unwrap();

        let mut subscriber = node
            .subscribe::<Joy>(topic_name, QosProfile::default())
            .unwrap();

        let last_joy_msg_clone = last_joy_msg.clone();

        tokio::spawn(async move {
            loop {
                println!("Running...");
                if let Some(joy) = subscriber.next().await {
                    let mut last_joy = last_joy_msg_clone.lock();
                    let ev = joy_difference_as_event(&last_joy, &joy);
                    *last_joy = joy;

                    tx.send(ev).unwrap();
                }
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
        });

        Self {
            last_joy_msg,
            _node: Arc::new(Mutex::new(node)),
            rx,
        }
    }
}

#[async_trait]
impl Gamepad for Ros2JoyGamepad {
    async fn next_event(&self) -> GamepadEvent {
        if let Ok(ev) = self.rx.recv_async().await {
            ev
        } else {
            GamepadEvent::Unknown
        }
    }

    fn stop(&self) {}
}

fn joy_difference_as_event(last_joy: &Joy, current_joy: &Joy) -> GamepadEvent {
    const DEAD_ZONE: f32 = 0.00001;
    todo!()
}
