use crate::error::Error;

pub type Wait<'a> = Box<dyn WaitTrait + 'a>;

#[must_use]
pub trait WaitTrait {
    fn wait(&self) -> Result<(), Error>;
}
