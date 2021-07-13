//! [`arci`] implementation using [urdf-viz](https://github.com/openrr/urdf-viz).

#![warn(rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

mod client;
mod utils;

pub use crate::client::*;
