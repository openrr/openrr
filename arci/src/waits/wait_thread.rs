use crate::error::Error;
use crate::traits::WaitTrait;
use std::{sync::Mutex, thread};

/// Waits for the associated thread to finish.
#[derive(Debug)]
pub struct WaitThread {
    handle: Mutex<Option<thread::JoinHandle<Result<(), Error>>>>,
}

impl WaitThread {
    /// Creates a new `WaitThread`.
    pub fn new(handle: thread::JoinHandle<Result<(), Error>>) -> Self {
        Self {
            handle: Mutex::new(Some(handle)),
        }
    }

    /// Creates a new boxed `WaitThread`.
    pub fn new_boxed(handle: thread::JoinHandle<Result<(), Error>>) -> Box<Self> {
        Box::new(Self::new(handle))
    }
}

impl WaitTrait for WaitThread {
    fn wait(&self) -> Result<(), Error> {
        if let Some(handle) = self.handle.lock().unwrap().take() {
            handle
                .join()
                .expect("Couldn't join on the associated thread")
        } else {
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_wait_thread() {
        let handle = thread::spawn(move || {
            thread::sleep(Duration::from_secs(1));
            Ok(())
        });
        let wait = WaitThread::new_boxed(handle);
        wait.wait().unwrap();
        // The second (and later) calls are no-op.
        wait.wait().unwrap();
    }
}
