use thiserror::Error;

#[derive(Debug, Error)]
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
    #[error("wait timeout target={:?}, cur={:?}", target, current)]
    TimeoutWithDiff { target: Vec<f64>, current: Vec<f64> },
    #[error("uninitialized : {}", message)]
    Uninitialized { message: String },
    #[error("connection error : {}", message)]
    Connection { message: String },
    #[error("arci: Other: {:?}", .0)]
    Other(#[from] anyhow::Error),
}
