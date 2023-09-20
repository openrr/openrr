use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::Duration,
};

use futures::{
    future::Future,
    stream::{Stream, StreamExt},
};

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

pub(crate) async fn wait(is_done: Arc<AtomicBool>) {
    loop {
        if is_done.load(Ordering::Relaxed) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

pub(crate) async fn subscribe_one<T: Send + 'static>(
    mut subscriber: impl Stream<Item = T> + Send + Unpin + 'static,
) -> Result<Option<T>, tokio::task::JoinError> {
    let is_done = Arc::new(AtomicBool::new(false));
    let is_done_clone = is_done.clone();
    let handle = tokio::spawn(async move {
        let next = subscriber.next().await;
        is_done.store(true, Ordering::Relaxed);
        next
    });
    wait(is_done_clone).await;
    handle.await
}
