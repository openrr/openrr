use std::{ops::RangeInclusive, sync::Arc};

use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("arci: {:?}", .0)]
    InterpolationError(String),
    #[error("arci: Collision {} {}", .0, .1)]
    CollisionError(String, String),
    #[error(
        "arci: Timeout {:?}: {} is larger than {}",
        timeout,
        allowable_total_diff,
        err
    )]
    Timeout {
        timeout: std::time::Duration,
        allowable_total_diff: f64,
        err: f64,
    },
    #[error("arci: Length mismatch (model = {}, input = {})", model, input)]
    LengthMismatch { model: usize, input: usize },
    #[error("arci: No Joint={} is found.", .0)]
    NoJoint(String),
    #[error(
        "arci: Joint Names Mismatch : left = {:?}, right = {:?}",
        partial,
        full
    )]
    JointNamesMismatch {
        partial: Vec<String>,
        full: Vec<String>,
    },
    #[error("arci: CopyJointError {:?} {:?} : {:?} {:?}", .0, .1, .2, .3)]
    CopyJointError(Vec<String>, Vec<f64>, Vec<String>, Vec<f64>),
    #[error(
        "arci: Wait timeout target={:?}, cur={:?} is_reached={:?}",
        target,
        current,
        is_reached
    )]
    TimeoutWithDiff {
        target: Vec<f64>,
        current: Vec<f64>,
        is_reached: Vec<bool>,
    },
    #[error("arci: Uninitialized : {}", message)]
    Uninitialized { message: String },
    #[error("arci: Connection error : {}", message)]
    Connection { message: String },
    #[error("arci: Canceled : {}", message)]
    Canceled { message: String },
    #[error(
        "arci: Out of limit: joint={}, position={}, limit={:?}",
        name,
        position,
        limit
    )]
    OutOfLimit {
        name: String,
        position: f64,
        limit: RangeInclusive<f64>,
    },
    #[error("arci: Failed to construct instance: {}", .0)]
    Lazy(Arc<Error>),
    #[error("arci: Other: {:?}", .0)]
    Other(#[from] anyhow::Error),
}
