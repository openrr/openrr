use std::path::PathBuf;
use thiserror::Error;
use urdf_rs::UrdfError;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("openrr-client: No JointTrajectoryClient={} is found.", .0)]
    NoJointTrajectoryClient(String),
    #[error("openrr-client: No IkClient={} is found.", .0)]
    NoIkClient(String),
    #[error("openrr-client: No Joint={} is found.", .0)]
    NoJoint(String),
    #[error("openrr-client: MismatchedLength {} != {}.", .0, .1)]
    MismatchedLength(usize, usize),
    #[error("openrr-client: No UrdfPath is specified.")]
    NoUrdfPath,
    #[error("openrr-client: No JointsPose {} {} is found.", .0, .1)]
    NoJointsPose(String, String),
    #[error("openrr-client: No ParentDirectory {:?} is found.", .0)]
    NoParentDirectory(PathBuf),
    #[error("openrr-client: arci: {:?}", .0)]
    Arci(#[from] arci::Error),
    #[error("openrr-client: urdf-rs: {:?}", .0)]
    UrdfRs(#[from] UrdfError),
}
