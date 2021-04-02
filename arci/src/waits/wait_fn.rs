use crate::error::Error;
use crate::traits::WaitTrait;

/// Waits for the closure to finish.
#[derive(Debug, Clone)]
pub struct WaitFn<F> {
    f: F,
}

impl<F> WaitFn<F>
where
    F: Fn() -> Result<(), Error>,
{
    /// Creates a new `WaitFn`.
    pub fn new(f: F) -> Self {
        Self { f }
    }

    /// Creates a new boxed `WaitFn`.
    pub fn new_boxed(f: F) -> Box<Self> {
        Box::new(Self::new(f))
    }
}

impl<F> WaitTrait for WaitFn<F>
where
    F: Fn() -> Result<(), Error>,
{
    fn wait(&self) -> Result<(), Error> {
        (self.f)()
    }
}
