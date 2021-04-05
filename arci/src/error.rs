use thiserror::Error;

/// Alias for a `Result` with the error type `arci::Error`.
pub type Result<T, E = Error> = std::result::Result<T, E>;

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
    #[error("length mismatch (model = {}, input = {})", model, input)]
    LengthMismatch { model: usize, input: usize },
    #[error(
        "wait timeout target={:?}, cur={:?} is_reached={:?}",
        target,
        current,
        is_reached
    )]
    TimeoutWithDiff {
        target: Vec<f64>,
        current: Vec<f64>,
        is_reached: Vec<bool>,
    },
    #[error("uninitialized : {}", message)]
    Uninitialized { message: String },
    #[error("connection error : {}", message)]
    Connection { message: String },
    #[error("arci: Other: {:?}", .0)]
    Other(#[from] anyhow::Error),
}
