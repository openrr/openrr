use super::control_node::ControlNode;
use arci::gamepad::{Button, Gamepad, GamepadEvent};
use arci::Speaker;
use std::sync::{
    atomic::{AtomicBool, AtomicUsize, Ordering},
    Arc, Mutex,
};
use std::thread;
use std::time::Duration;
use tracing::{debug, warn};

pub struct ControlNodeSwitcher<N, S>
where
    N: ControlNode,
    S: Speaker,
{
    current_index: Arc<AtomicUsize>,
    control_nodes: Arc<Mutex<Vec<N>>>,
    speaker: S,
    is_running: Arc<AtomicBool>,
}

impl<N, S> ControlNodeSwitcher<N, S>
where
    N: 'static + ControlNode,
    S: Speaker,
{
    pub fn new(control_nodes: Vec<N>, speaker: S, initial_node_index: usize) -> Self {
        assert!(!control_nodes.is_empty());
        Self {
            current_index: Arc::new(AtomicUsize::new(initial_node_index)),
            control_nodes: Arc::new(Mutex::new(control_nodes)),
            speaker,
            is_running: Arc::new(AtomicBool::new(false)),
        }
    }
    pub fn increment_mode(&self) {
        let len = self.control_nodes.lock().unwrap().len();
        self.current_index.fetch_add(1, Ordering::Relaxed);
        let next = self.current_index.load(Ordering::Relaxed) % len;
        self.current_index.store(next, Ordering::Relaxed);
        self.speak_current_mode();
    }
    pub fn speak_current_mode(&self) {
        let nodes = self.control_nodes.lock().unwrap();
        let i = self.current_index();
        let mode = nodes[i].mode();
        let submode = nodes[i].submode();
        self.speaker.speak(&format!("{}{}", mode, submode,));
    }
    fn current_index(&self) -> usize {
        self.current_index.load(Ordering::Relaxed)
    }
    fn is_running(&self) -> bool {
        self.is_running.load(Ordering::Relaxed)
    }
    pub fn stop(&self) {
        self.is_running.store(false, Ordering::Relaxed);
    }
    pub fn main<G>(&self, gamepad: G)
    where
        G: 'static + Gamepad,
    {
        let nodes = self.control_nodes.clone();
        let index = self.current_index.clone();
        let is_running = self.is_running.clone();
        self.is_running.store(true, Ordering::Relaxed);
        self.speak_current_mode();
        let gamepad = Arc::new(gamepad);
        let gamepad_cloned = gamepad.clone();
        thread::spawn(move || {
            let interval = Duration::from_millis(50);
            while is_running.load(Ordering::Relaxed) {
                debug!("tick");
                nodes.lock().unwrap()[index.load(Ordering::Relaxed)].proc();
                thread::sleep(interval);
            }
            gamepad_cloned.stop();
        });
        while self.is_running() {
            let ev = gamepad.next_event();
            debug!("event: {:?}", ev);
            match ev {
                GamepadEvent::ButtonPressed(Button::North) => {
                    self.increment_mode();
                }
                GamepadEvent::Unknown => {
                    warn!("gamepad Unkwon");
                    self.stop();
                }
                _ => {
                    self.control_nodes.lock().unwrap()[self.current_index()].set_event(ev);
                }
            }
        }
    }
}
