use std::path::PathBuf;
use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("openrr-client: No IkClient={} is found.", .0)]
    NoIkClient(String),
    #[error("openrr-client: Failed to open {:?} {:?}.", .0, .1)]
    CommandFileOpenFailure(PathBuf, String),
    #[error("openrr-client: No Command is specified {:?}.", .0)]
    NoCommand(Vec<String>),
    #[error("openrr-client: Failed to execute Command {:?} ({}).", .0, .1)]
    CommandExecutionFailure(Vec<String>, #[source] std::io::Error),
    #[error("openrr-client: arci: {:?}", .0)]
    Arci(#[from] arci::Error),
    #[error("openrr-client: openrr-client: {:?}", .0)]
    OpenrrClient(#[from] openrr_client::Error),
}
