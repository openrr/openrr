use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

use parking_lot::{Mutex, MutexGuard};

/// ROS2 node. This is a wrapper around `Arc<Mutex<r2r::Node>>`.
#[derive(Clone)]
pub struct Node {
    inner: Arc<NodeInner>,
}

struct NodeInner {
    node: Mutex<r2r::Node>,
    has_spin_thread: AtomicBool,
}

impl Node {
    /// Creates a new ROS2 node.
    pub fn new(name: &str, namespace: &str) -> Result<Self, arci::Error> {
        let ctx = r2r::Context::create().map_err(anyhow::Error::from)?;
        Self::with_context(ctx, name, namespace)
    }

    /// Creates a new ROS2 node with `r2r::Context`.
    pub fn with_context(
        ctx: r2r::Context,
        name: &str,
        namespace: &str,
    ) -> Result<Self, arci::Error> {
        let node = r2r::Node::create(ctx, name, namespace).map_err(anyhow::Error::from)?;
        Ok(Self {
            inner: Arc::new(NodeInner {
                node: Mutex::new(node),
                has_spin_thread: AtomicBool::new(false),
            }),
        })
    }

    /// Gets underlying `r2r::Node`.
    pub fn r2r(&self) -> MutexGuard<'_, r2r::Node> {
        self.inner.node.lock()
    }

    /// Creates a thread to spin the ROS2 node.
    pub fn run_spin_thread(&self, interval: Duration) {
        if self.inner.has_spin_thread.swap(true, Ordering::Relaxed) {
            return;
        }
        let node = self.clone();
        tokio::spawn(async move {
            while Arc::strong_count(&node.inner) > 1 {
                node.spin_once(interval).await;
            }
        });
    }

    /// Spins the ROS2 node.
    pub async fn spin_once(&self, duration: Duration) {
        let now = std::time::Instant::now();
        // Sleep with tokio::time::sleep instead of spin_once, since spin_once
        // blocks the current thread while still holding the lock.
        self.r2r().spin_once(Duration::ZERO);
        tokio::time::sleep(duration.saturating_sub(now.elapsed())).await;
    }
}
