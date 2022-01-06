#![doc = include_str!("../README.md")]
#![warn(missing_docs, rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

mod overwrite;
pub use overwrite::{overwrite, overwrite_str};

mod evaluate;
pub use evaluate::{evaluate, evaluate_bash};
