use std::path::PathBuf;
use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("openrr-apps: No ConfigPath is specified.")]
    NoConfigPath,
    #[error("openrr-apps: Failed to parse {:?} as toml ({}).", .0, .1)]
    TomlParseFailure(PathBuf, #[source] toml::de::Error),
    #[error("openrr-apps: No File {:?} is found ({}).", .0, .1)]
    NoFile(PathBuf, #[source] std::io::Error),
    #[error("openrr-apps: No ParentDirectory {:?} is found.", .0)]
    NoParentDirectory(PathBuf),
    #[error("openrr-apps: openrr-client: {:?}", .0)]
    OpenrrClient(#[from] openrr_client::Error),
    #[error("openrr-apps: openrr-command: {:?}", .0)]
    OpenrrCommand(#[from] openrr_command::Error),
}
