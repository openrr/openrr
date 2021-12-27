use std::thread;

use futures::future::Future;

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
