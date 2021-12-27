#![doc = include_str!("../README.md")]
#![warn(rust_2018_idioms)]
#![cfg(feature = "ros2")]
#![warn(
    future_incompatible,
    missing_docs,
    rust_2018_idioms,
    single_use_lifetimes,
    unreachable_pub
)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

mod cmd_vel_move_base;
mod navigation;
mod plugin;
mod ros2_control;
mod utils;

pub use cmd_vel_move_base::*;
pub use navigation::*;
// re-export
pub use r2r;
pub use ros2_control::*;
