use std::path::PathBuf;

use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("openrr-command: No IkClient={} is found.", .0)]
    NoIkClient(String),
    #[error("openrr-command: Failed to open {:?} {:?}.", .0, .1)]
    CommandFileOpenFailure(PathBuf, String),
    #[error("openrr-command: No Command is specified {:?}.", .0)]
    NoCommand(Vec<String>),
    #[error("openrr-command: Failed to execute Command {:?} ({}).", .0, .1)]
    CommandExecutionFailure(Vec<String>, #[source] std::io::Error),
    #[error("openrr-command: Command {:?} Error ({:?}).", .0, .1)]
    CommandFailure(Vec<String>, String),
    #[error("openrr-command: arci: {:?}", .0)]
    Arci(#[from] arci::Error),
    #[error("openrr-command: openrr-client: {:?}", .0)]
    OpenrrClient(#[from] openrr_client::Error),
}
