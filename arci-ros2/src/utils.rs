use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::Duration,
};

use futures::future::Future;
use parking_lot::Mutex;

// https://github.com/openrr/openrr/pull/501#discussion_r746183161
pub(crate) fn spawn_blocking<T: Send + 'static>(
    future: impl Future<Output = T> + Send + 'static,
) -> thread::JoinHandle<T> {
    std::thread::spawn(move || {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(future)
    })
}

pub(crate) async fn spin(is_done: Arc<AtomicBool>, node: Arc<Mutex<r2r::Node>>) {
    loop {
        if is_done.load(Ordering::Relaxed) {
            break;
        }
        // Sleep with tokio::time instead of spin_once, since spin_once blocks the current thread.
        node.lock().spin_once(Duration::ZERO);
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}
