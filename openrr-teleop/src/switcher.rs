use super::control_node::ControlNode;
use arci::gamepad::{Button, Gamepad, GamepadEvent};
use arci::Speaker;
use log::{debug, warn};
use std::sync::{
    atomic::{AtomicBool, AtomicUsize, Ordering},
    Arc,
};
use std::time::Duration;
use tokio::sync::Mutex as TokioMutex;

pub struct ControlNodeSwitcher<N, S>
where
    N: ControlNode,
    S: Speaker,
{
    current_index: Arc<AtomicUsize>,
    control_nodes: Arc<TokioMutex<Vec<N>>>,
    speaker: Arc<S>,
    is_running: Arc<AtomicBool>,
}

impl<N, S> ControlNodeSwitcher<N, S>
where
    N: 'static + ControlNode,
    S: 'static + Speaker + Send + Sync,
{
    pub fn new(control_nodes: Vec<N>, speaker: S) -> Self {
        assert!(!control_nodes.is_empty());
        Self {
            current_index: Arc::new(AtomicUsize::new(0)),
            control_nodes: Arc::new(TokioMutex::new(control_nodes)),
            speaker: Arc::new(speaker),
            is_running: Arc::new(AtomicBool::new(false)),
        }
    }
    pub async fn increment_mode(&self) {
        let len = self.control_nodes.lock().await.len();
        self.current_index.fetch_add(1, Ordering::Relaxed);
        let next = self.current_index.load(Ordering::Relaxed) % len;
        self.current_index.store(next, Ordering::Relaxed);
        self.speak_current_mode().await;
    }
    pub async fn speak_current_mode(&self) {
        let nodes = self.control_nodes.lock().await;
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
    pub async fn main<G>(&self, gamepad: G)
    where
        G: 'static + Gamepad + Send + Sync,
    {
        let nodes = self.control_nodes.clone();
        let index = self.current_index.clone();
        let is_running = self.is_running.clone();
        self.is_running.store(true, Ordering::Relaxed);
        self.speak_current_mode().await;
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(50));
            while is_running.load(Ordering::Relaxed) {
                debug!("tick");
                nodes.lock().await[index.load(Ordering::Relaxed)]
                    .proc()
                    .await;
                interval.tick().await;
            }
        });
        while self.is_running() {
            let ev = gamepad.next_event().await;
            debug!("event: {:?}", ev);
            match ev {
                GamepadEvent::ButtonPressed(Button::North) => {
                    self.increment_mode().await;
                }
                GamepadEvent::Unknown => {
                    warn!("gamepad Unkwon");
                    self.stop();
                }
                _ => {
                    self.control_nodes.lock().await[self.current_index()].set_event(ev);
                }
            }
        }
    }
}
