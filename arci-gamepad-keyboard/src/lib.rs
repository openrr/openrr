#![doc = include_str!("../README.md")]
#![cfg(unix)]
#![warn(missing_docs, rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

use std::{
    collections::HashMap,
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
    key_state: HashMap<char, bool>,
    button_map: HashMap<char, Button>,
}

#[rustfmt::skip]
const LEFT_STICK_KEYS: &[char] = &[
    'q', 'w', 'e',
    'a', 's', 'd',
    'z', 'x', 'c',
];
#[rustfmt::skip]
const RIGHT_STICK_KEYS: &[char] = &[
    'u', 'i', 'o',
    'j', 'k', 'l',
    'm', ',', '.',
];
const DEFAULT_AXIS_VALUE: f64 = 0.3;

impl State {
    fn new(sender: flume::Sender<GamepadEvent>) -> Self {
        let mut key_state = HashMap::new();
        for ch in ('0'..='9').chain('a'..='z') {
            key_state.insert(ch, false);
        }
        key_state.insert(',', false);
        key_state.insert('.', false);

        Self {
            sender,
            key_state,
            button_map: button_map(),
        }
    }

    fn right_stick(&mut self, sign_x: i8, sign_y: i8) {
        self.send(GamepadEvent::AxisChanged(
            Axis::RightStickX,
            sign_x as f64 * DEFAULT_AXIS_VALUE,
        ));
        self.send(GamepadEvent::AxisChanged(
            Axis::RightStickY,
            sign_y as f64 * DEFAULT_AXIS_VALUE,
        ));
    }

    fn left_stick(&mut self, sign_x: i8, sign_y: i8) {
        self.send(GamepadEvent::AxisChanged(
            Axis::LeftStickX,
            sign_x as f64 * DEFAULT_AXIS_VALUE,
        ));
        self.send(GamepadEvent::AxisChanged(
            Axis::LeftStickY,
            sign_y as f64 * DEFAULT_AXIS_VALUE,
        ));
    }

    fn send_left_stick(&mut self, ch: char) {
        match ch {
            'q' => {
                self.left_stick(1, 1);
            }
            'w' => {
                self.left_stick(0, 1);
            }
            'e' => {
                self.left_stick(-1, 1);
            }
            'a' => {
                self.left_stick(1, 0);
            }
            's' => {
                self.left_stick(0, 0);
            }
            'd' => {
                self.left_stick(-1, 0);
            }
            'z' => {
                self.left_stick(1, -1);
            }
            'x' => {
                self.left_stick(0, -1);
            }
            'c' => {
                self.left_stick(-1, -1);
            }
            _ => unreachable!(),
        }
    }

    fn send_right_stick(&mut self, ch: char) {
        match ch {
            'u' => {
                self.right_stick(1, 1);
            }
            'i' => {
                self.right_stick(0, 1);
            }
            'o' => {
                self.right_stick(-1, 1);
            }
            'j' => {
                self.right_stick(1, 0);
            }
            'k' => {
                self.right_stick(0, 0);
            }
            'l' => {
                self.right_stick(-1, 0);
            }
            'm' => {
                self.right_stick(1, -1);
            }
            ',' => {
                self.right_stick(0, -1);
            }
            '.' => {
                self.right_stick(-1, -1);
            }
            _ => unreachable!(),
        }
    }

    fn send_button(&mut self, ch: char) {
        if let Some(&button) = self.button_map.get(&ch) {
            let active = self.key_state.get_mut(&ch).unwrap();
            *active = !*active;
            if *active {
                self.send(GamepadEvent::ButtonPressed(button));
            } else {
                self.send(GamepadEvent::ButtonReleased(button));
            }
        }
    }

    fn send(&self, event: GamepadEvent) {
        debug!("sending {event:?}");
        if let Err(e) = self.sender.send(event) {
            error!("{e}");
        }
    }

    fn send_event(&mut self, ch: char) {
        if LEFT_STICK_KEYS.contains(&ch) {
            self.send_left_stick(ch);
        } else if RIGHT_STICK_KEYS.contains(&ch) {
            self.send_right_stick(ch);
        } else {
            self.send_button(ch);
        }
    }
}

fn button_map() -> HashMap<char, Button> {
    let mut map = HashMap::new();
    map.insert('1', Button::LeftTrigger);
    map.insert('2', Button::LeftTrigger2);
    map.insert('3', Button::LeftThumb);

    map.insert('6', Button::Select);
    map.insert('7', Button::Start);

    map.insert('8', Button::RightTrigger);
    map.insert('9', Button::RightTrigger2);
    map.insert('0', Button::RightThumb);

    // <^>v
    map.insert('5', Button::DPadUp);
    map.insert('r', Button::DPadLeft);
    map.insert('t', Button::DPadRight);
    map.insert('f', Button::DPadDown);

    // △○□x
    map.insert('y', Button::North);
    map.insert('g', Button::West);
    map.insert('h', Button::East);
    map.insert('b', Button::South);

    map
}

/// [`arci::Gamepad`] implementation for keyboard.
pub struct KeyboardGamepad {
    receiver: flume::Receiver<GamepadEvent>,
    is_running: Arc<AtomicBool>,
}

impl KeyboardGamepad {
    /// Creates a new `KeyboardGamepad`.
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
        let mut reader = io::stdin();
        io::stdout().lock().flush().unwrap();
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
                debug!("non-ascii input: {b}");
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
                error!("recv error: {e}");
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

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    const TIMEOUT: Duration = Duration::from_secs(1);

    #[test]
    fn button() {
        let (sender, receiver) = flume::unbounded();
        let mut state = State::new(sender);
        let button_map = button_map();

        for (&ch, button) in &button_map {
            state.send_event(ch);
            assert!(
                matches!(receiver.recv_timeout(TIMEOUT).unwrap(), GamepadEvent::ButtonPressed(b) if b == *button)
            );
        }
        for (&ch, button) in &button_map {
            state.send_event(ch);
            assert!(
                matches!(receiver.recv_timeout(TIMEOUT).unwrap(), GamepadEvent::ButtonReleased(b) if b == *button)
            );
        }
    }

    #[test]
    fn axis() {
        let (sender, receiver) = flume::unbounded();
        let mut state = State::new(sender);

        for &ch in LEFT_STICK_KEYS {
            state.send_event(ch);
            assert!(matches!(
                receiver.recv_timeout(TIMEOUT).unwrap(),
                GamepadEvent::AxisChanged(Axis::LeftStickX, _)
            ));
            assert!(matches!(
                receiver.recv_timeout(TIMEOUT).unwrap(),
                GamepadEvent::AxisChanged(Axis::LeftStickY, _)
            ));
        }
        for &ch in RIGHT_STICK_KEYS {
            state.send_event(ch);
            assert!(matches!(
                receiver.recv_timeout(TIMEOUT).unwrap(),
                GamepadEvent::AxisChanged(Axis::RightStickX, _)
            ));
            assert!(matches!(
                receiver.recv_timeout(TIMEOUT).unwrap(),
                GamepadEvent::AxisChanged(Axis::RightStickY, _)
            ));
        }
    }
}
