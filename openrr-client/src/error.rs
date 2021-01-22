use std::path::PathBuf;
use thiserror::Error;
use urdf_rs::UrdfError;

#[derive(Debug, Error)]
pub enum Error {
    #[error("openrr-cleint: No JointTrajectoryClient={} is found.", .0)]
    NoJointTrajectoryClient(String),
    #[error("openrr-cleint: No IkClient={} is found.", .0)]
    NoIkClient(String),
    #[error("openrr-cleint: No Joint={} is found.", .0)]
    NoJoint(String),
    #[error("openrr-cleint: MismatchedLength {} != {}.", .0, .1)]
    MismatchedLength(usize, usize),
    #[error("openrr-cleint: No UrdfPath is specified.")]
    NoUrdfPath,
    #[error("openrr-cleint: No JointsPose {} {} is found.", .0, .1)]
    NoJointsPose(String, String),
    #[error("openrr-cleint: No ParentDirectory {:?} is found.", .0)]
    NoParentDirectory(PathBuf),
    #[error("openrr-cleint: arci: {:?}", .0)]
    Arci(#[from] arci::Error),
    #[error("openrr-cleint: urdf-rs: {:?}", .0)]
    UrdfRs(#[from] UrdfError),
}
