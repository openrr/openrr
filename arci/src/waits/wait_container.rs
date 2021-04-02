use crate::error::Error;
use crate::traits::{Wait, WaitTrait};

pub struct WaitContainer<'a> {
    waits: Vec<Wait<'a>>,
}

impl<'a> WaitContainer<'a> {
    /// Creates a new `WaitContainer`.
    pub fn new(waits: Vec<Wait<'a>>) -> Self {
        Self { waits }
    }

    /// Creates a new boxed `WaitContainer`.
    pub fn new_boxed(waits: Vec<Wait<'a>>) -> Box<Self> {
        Box::new(Self::new(waits))
    }
}

impl WaitTrait for WaitContainer<'_> {
    fn wait(&self) -> Result<(), Error> {
        for wait in &self.waits {
            wait.wait()?;
        }
        Ok(())
    }
}
