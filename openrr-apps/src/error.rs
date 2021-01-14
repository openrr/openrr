use std::path::PathBuf;
use thiserror::Error;
use urdf_rs::UrdfError;

#[derive(Debug, Error)]
pub enum Error {
    #[error("openrr-apps: No RawJointTrajectoryClient={} is found.", .0)]
    NoRawJointTrajectoryClient(String),
    #[error("openrr-apps: No JointTrajectoryClient={} is found.", .0)]
    NoJointTrajectoryClient(String),
    #[error("openrr-apps: No CollisionCheckClient={} is found.", .0)]
    NoCollisionCheckClient(String),
    #[error("openrr-apps: No IkClient={} is found.", .0)]
    NoIkClient(String),
    #[error("openrr-apps: No Joint={} is found.", .0)]
    NoJoint(String),
    #[error("openrr-apps: MismatchedLength {} != {}.", .0, .1)]
    MismatchedLength(usize, usize),
    #[error("openrr-apps: Failed to open {:?} {:?}.", .0, .1)]
    CommandFileOpenFailure(PathBuf, String),
    #[error("openrr-apps: No ConfigPath is specified.")]
    NoConfigPath,
    #[error("openrr-apps: No ClientsConfigs for {} is specified.", .0)]
    NoClientsConfigs(String),
    #[error("openrr-apps: No UrdfPath is specified.")]
    NoUrdfPath,
    #[error("openrr-apps: Failed to parse {:?} as toml ({}).", .0, .1)]
    TomlParseFailure(PathBuf, #[source] toml::de::Error),
    #[error("openrr-apps: No File {:?} is found ({}).", .0, .1)]
    NoFile(PathBuf, #[source] std::io::Error),
    #[error("openrr-apps: No JointsPose {} {} is found.", .0, .1)]
    NoJointsPose(String, String),
    #[error("openrr-apps: No Command is specified {:?}.", .0)]
    NoCommand(Vec<String>),
    #[error("openrr-apps: Failed to execute Command {:?} ({}).", .0, .1)]
    CommandExecutionFailure(Vec<String>, #[source] std::io::Error),
    #[error("openrr-apps: No ParentDirectory {:?} is found.", .0)]
    NoParentDirectory(PathBuf),
    #[error("openrr-apps: arci: {:?}", .0)]
    Arci(#[from] arci::Error),
    #[error("openrr-apps: urdf-rs: {:?}", .0)]
    UrdfRs(#[from] UrdfError),
}
