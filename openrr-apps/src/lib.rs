#![warn(rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

mod error;
mod robot_config;
mod robot_teleop_config;
pub mod utils;

pub use crate::{error::*, robot_config::*, robot_teleop_config::*};
