use crate::error::Error;
use crate::traits::WaitTrait;

#[derive(Debug, Clone, Default)]
pub struct DummyWait {}

impl DummyWait {
    /// Creates a new `DummyWait`.
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates a new boxed `DummyWait`.
    pub fn new_boxed() -> Box<Self> {
        Box::new(Self::default())
    }
}

impl WaitTrait for DummyWait {
    fn wait(&self) -> Result<(), Error> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dummy_wait() {
        let d = DummyWait {};
        assert!(d.wait().is_ok());
    }
    #[test]
    fn test_dummy_wait_boxed() {
        let d = DummyWait::new_boxed();
        assert!(d.wait().is_ok());
    }
}
