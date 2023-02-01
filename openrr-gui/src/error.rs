use thiserror::Error;

#[derive(Error, Debug)]
#[non_exhaustive]
pub enum Error {
    #[error("openrr-gui: arci: {}", .0)]
    Arci(#[from] arci::Error),
    #[error("openrr-gui: other: {}", .0)]
    Other(String),
}
