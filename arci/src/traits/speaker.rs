use auto_impl::auto_impl;

use crate::{error::Error, waits::WaitFuture};

#[auto_impl(Box, Arc)]
pub trait Speaker: Send + Sync {
    /// Starts speaking and returns a future that waits until complete the speaking.
    ///
    /// # Implementation
    ///
    /// The returned future is expected to behave similarly to
    /// [`std::thread::JoinHandle`] and [`tokio::task::JoinHandle`]:
    ///
    /// - Can wait for the operation to complete by `.await`.
    /// - The operation does not end even if it is dropped.
    ///
    /// If the operation may block the current thread for an extended period of
    /// time, consider [spawning a thread to running blocking
    /// operations](https://docs.rs/tokio/1/tokio/index.html#cpu-bound-tasks-and-blocking-code).
    fn speak(&self, message: &str) -> Result<WaitFuture, Error>;
}
