use std::{
    sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        Arc,
    },
    time::Duration,
};

use arci::{
    gamepad::{Button, Gamepad, GamepadEvent},
    Speaker,
};
use tokio::sync::Mutex as TokioMutex;
use tracing::{debug, warn};

use super::control_node::ControlNode;

pub struct ControlNodeSwitcher<N, S>
where
    N: ControlNode,
    S: Speaker,
{
    current_index: Arc<AtomicUsize>,
    control_nodes: Arc<TokioMutex<Vec<N>>>,
    speaker: S,
    is_running: Arc<AtomicBool>,
}

impl<N, S> ControlNodeSwitcher<N, S>
where
    N: 'static + ControlNode,
    S: Speaker,
{
    #[track_caller]
    pub fn new(control_nodes: Vec<N>, speaker: S, initial_node_index: usize) -> Self {
        assert!(!control_nodes.is_empty());
        Self {
            current_index: Arc::new(AtomicUsize::new(initial_node_index)),
            control_nodes: Arc::new(TokioMutex::new(control_nodes)),
            speaker,
            is_running: Arc::new(AtomicBool::new(false)),
        }
    }

    pub async fn increment_mode(&self) -> Result<(), arci::Error> {
        let len = self.control_nodes.lock().await.len();
        self.current_index.fetch_add(1, Ordering::Relaxed);
        let next = self.current_index.load(Ordering::Relaxed) % len;
        self.current_index.store(next, Ordering::Relaxed);
        self.speak_current_mode().await
    }

    pub async fn speak_current_mode(&self) -> Result<(), arci::Error> {
        let nodes = self.control_nodes.lock().await;
        let i = self.current_index();
        let mode = nodes[i].mode();
        let submode = nodes[i].submode();
        self.speaker.speak(&format!("{}{}", mode, submode))?.await
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

    pub async fn main<G>(&self, gamepad: G)
    where
        G: 'static + Gamepad,
    {
        let nodes = self.control_nodes.clone();
        let index = self.current_index.clone();
        let is_running = self.is_running.clone();
        self.is_running.store(true, Ordering::Relaxed);
        self.speak_current_mode().await.unwrap();
        let gamepad = Arc::new(gamepad);
        let gamepad_cloned = gamepad.clone();
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(50));
            while is_running.load(Ordering::Relaxed) {
                debug!("tick");
                nodes.lock().await[index.load(Ordering::Relaxed)]
                    .proc()
                    .await;
                interval.tick().await;
            }
            gamepad_cloned.stop();
        });
        while self.is_running() {
            let ev = gamepad.next_event().await;
            debug!("event: {:?}", ev);
            match ev {
                GamepadEvent::ButtonPressed(Button::North) => {
                    self.increment_mode().await.unwrap();
                }
                GamepadEvent::Unknown => {
                    warn!("gamepad Unknown");
                    self.stop();
                }
                _ => {
                    self.control_nodes.lock().await[self.current_index()].set_event(ev);
                }
            }
        }
    }
}
