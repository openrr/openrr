//! [`arci::Gamepad`] implementation for keyboard.
//!
//! Currently, this crate only supports Unix-like operating systems.

#![cfg(unix)]

use std::{
    io::{self, Read, Write},
    sync::{
        atomic::{AtomicBool, Ordering::Relaxed},
        Arc,
    },
};

use arci::{gamepad::*, *};
use termios::{tcsetattr, Termios};
use tracing::{debug, error};

#[derive(Debug)]
struct State {
    sender: flume::Sender<GamepadEvent>,
    enabled: bool,
    right_stick_x: i8,
    right_stick_y: i8,
    left_stick_x: i8,
    left_stick_y: i8,
}

impl State {
    fn new(sender: flume::Sender<GamepadEvent>) -> Self {
        Self {
            sender,
            enabled: false,
            right_stick_x: 0,
            right_stick_y: 0,
            left_stick_x: 0,
            left_stick_y: 0,
        }
    }

    fn enable(&mut self) {
        if !self.enabled {
            self.enabled = true;
            self.send(GamepadEvent::ButtonPressed(Button::RightTrigger2));
        }
    }

    fn disable(&mut self) {
        if self.enabled {
            self.enabled = false;
            self.send(GamepadEvent::ButtonReleased(Button::RightTrigger2));
        }
    }

    /// joints mode: (none)
    /// base mode: vel.theta
    fn right_stick_x(&mut self, v: i8) {
        if self.right_stick_x != v {
            self.right_stick_x = v;
            self.send(GamepadEvent::AxisChanged(Axis::RightStickX, v as f64 * 0.3));
        }
    }

    /// joints mode: move joint
    /// base mode: (none)
    fn right_stick_y(&mut self, v: i8) {
        if self.right_stick_y != v {
            self.right_stick_y = v;
            self.send(GamepadEvent::AxisChanged(Axis::RightStickY, v as f64 * 0.3));
        }
    }

    /// joints mode: (none)
    /// base mode: vel.y
    fn left_stick_x(&mut self, v: i8) {
        if self.left_stick_x != v {
            self.left_stick_x = v;
            self.send(GamepadEvent::AxisChanged(Axis::LeftStickX, v as f64 * 0.3));
        }
    }

    /// joints mode: (none)
    /// base mode: vel.x
    fn left_stick_y(&mut self, v: i8) {
        if self.left_stick_y != v {
            self.left_stick_y = v;
            self.send(GamepadEvent::AxisChanged(Axis::LeftStickY, v as f64 * 0.3));
        }
    }

    fn send(&mut self, event: GamepadEvent) {
        let _ = self.sender.send(event);
    }

    fn send_event(&mut self, c: char) {
        match c {
            // North
            'w' => {
                self.disable();
                self.send(GamepadEvent::ButtonPressed(Button::North));
            }
            // East
            'd' => {
                self.disable();
                self.send(GamepadEvent::ButtonPressed(Button::East));
            }
            // West
            'a' => {
                self.send(GamepadEvent::ButtonPressed(Button::West));
                self.enable();
            }
            // South
            's' => {
                self.send(GamepadEvent::ButtonPressed(Button::South));
                self.enable();
            }
            'q' => {
                self.send(GamepadEvent::ButtonPressed(Button::LeftTrigger2));
            }
            'z' => {
                self.send(GamepadEvent::ButtonReleased(Button::LeftTrigger2));
            }

            // for joint mode
            'x' => {
                self.right_stick_y(1);
                self.enable();
            }
            'c' => {
                self.right_stick_y(0);
                self.disable();
            }
            'v' => {
                self.right_stick_y(-1);
                self.enable();
            }

            // for base mode
            // key mapping is based on teleop_twist_keyboard:
            // http://wiki.ros.org/stdr_simulator/Tutorials/Teleop%20with%20teleop_twist_keyboard
            'i' | 'I' => {
                self.left_stick_y(1);
                self.left_stick_x(0);
                self.right_stick_x(0);
                self.enable();
            }
            'o' => {
                self.left_stick_y(1);
                self.left_stick_x(0);
                self.right_stick_x(-1);
                self.enable();
            }
            'j' => {
                self.left_stick_y(0);
                self.left_stick_x(0);
                self.right_stick_x(1);
                self.enable();
            }
            'l' => {
                self.left_stick_y(0);
                self.left_stick_x(0);
                self.right_stick_x(-1);
                self.enable();
            }
            'u' => {
                self.left_stick_y(1);
                self.left_stick_x(0);
                self.right_stick_x(1);
                self.enable();
            }
            ',' | '<' => {
                self.left_stick_y(-1);
                self.left_stick_x(0);
                self.right_stick_x(0);
                self.enable();
            }
            '.' => {
                self.left_stick_y(-1);
                self.left_stick_x(0);
                self.right_stick_x(1);
                self.enable();
            }
            'm' => {
                self.left_stick_y(-1);
                self.left_stick_x(0);
                self.right_stick_x(-1);
                self.enable();
            }
            'O' => {
                self.left_stick_y(1);
                self.left_stick_x(-1);
                self.right_stick_x(0);
                self.enable();
            }
            'J' => {
                self.left_stick_y(0);
                self.left_stick_x(1);
                self.right_stick_x(0);
                self.enable();
            }
            'L' => {
                self.left_stick_y(0);
                self.left_stick_x(-1);
                self.right_stick_x(0);
                self.enable();
            }
            'U' => {
                self.left_stick_y(1);
                self.left_stick_x(1);
                self.right_stick_x(0);
                self.enable();
            }
            '>' => {
                self.left_stick_y(-1);
                self.left_stick_x(-1);
                self.right_stick_x(0);
                self.enable();
            }
            'M' => {
                self.left_stick_y(-1);
                self.left_stick_x(1);
                self.right_stick_y(0);
                self.right_stick_x(0);
                self.enable();
            }
            'k' => {
                self.disable();
            }

            // TODO: support ik mode.
            _ => {}
        }
    }
}

/*

fn default_axis_value_map() -> HashMap<Axis, f64> {
    let mut axis_value_map = HashMap::new();
    axis_value_map.insert(Axis::RightStickX, -1.0);
    axis_value_map.insert(Axis::LeftStickX, -1.0);
    axis_value_map
}
*/

pub struct KeyboardGamepad {
    receiver: flume::Receiver<GamepadEvent>,
    is_running: Arc<AtomicBool>,
}

impl KeyboardGamepad {
    pub fn new() -> Self {
        let (sender, receiver) = flume::unbounded();
        let is_running = Arc::new(AtomicBool::new(true));
        let is_running_cloned = is_running.clone();

        // Based on https://stackoverflow.com/questions/26321592/how-can-i-read-one-character-from-stdin-without-having-to-hit-enter
        let stdin = 0; // couldn't get std::os::unix::io::FromRawFd to work
                       // on /dev/stdin or /dev/tty
        let termios = Termios::from_fd(stdin).unwrap();
        let mut new_termios = termios; // make a mutable copy of termios
                                       // that we will modify
        new_termios.c_lflag &= !(termios::ICANON | termios::ECHO); // no echo and canonical mode
        tcsetattr(stdin, termios::TCSANOW, &new_termios).unwrap();
        let stdout = io::stdout();
        let mut reader = io::stdin();
        stdout.lock().flush().unwrap();
        drop(stdout);
        std::thread::spawn(move || {
            let mut state = State::new(sender);
            while is_running_cloned.load(Relaxed) {
                let mut buffer = [0; 1]; // read exactly one byte
                reader.read_exact(&mut buffer).unwrap();
                let b = buffer[0];
                if b.is_ascii() {
                    state.send_event(b as char);
                    continue;
                }
                debug!("non-ascii input: {}", b);
            }
            tcsetattr(stdin, termios::TCSANOW, &termios).unwrap(); // reset the stdin to
        });

        Self {
            receiver,
            is_running,
        }
    }
}

impl Default for KeyboardGamepad {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Gamepad for KeyboardGamepad {
    async fn next_event(&self) -> GamepadEvent {
        match self.receiver.recv_async().await {
            Ok(e) => e,
            Err(e) => {
                error!("recv error: {}", e);
                GamepadEvent::Unknown
            }
        }
    }

    fn stop(&self) {
        self.is_running.store(false, Relaxed);
    }
}

impl Drop for KeyboardGamepad {
    fn drop(&mut self) {
        self.stop();
    }
}
