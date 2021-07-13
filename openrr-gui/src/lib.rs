#![warn(rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

mod error;
mod joint_position_sender;
mod style;
mod velocity_sender;

pub use crate::{error::*, joint_position_sender::*, velocity_sender::*};
