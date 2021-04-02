use crate::error::Error;
use crate::traits::WaitTrait;

#[derive(Debug, Clone, Default)]
pub struct SleepWait {
    sleep_duration: std::time::Duration,
}

impl SleepWait {
    /// Creates a new `SleepWait`.
    pub fn new(sleep_duration: std::time::Duration) -> Self {
        Self { sleep_duration }
    }

    /// Creates a new boxed `SleepWait`.
    pub fn new_boxed(sleep_duration: std::time::Duration) -> Box<Self> {
        Box::new(Self::new(sleep_duration))
    }
}

impl WaitTrait for SleepWait {
    fn wait(&self) -> Result<(), Error> {
        std::thread::sleep(self.sleep_duration);
        Ok(())
    }
}
