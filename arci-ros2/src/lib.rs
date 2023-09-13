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
// buggy: https://github.com/rust-lang/rust-clippy/issues?q=is%3Aissue+derive_partial_eq_without_eq
#![allow(clippy::derive_partial_eq_without_eq)]

mod cmd_vel_move_base;
mod navigation;
mod plugin;
mod ros2_control;
mod ros2_laser_scan;
mod utils;

pub use cmd_vel_move_base::*;
pub use navigation::*;
// re-export
pub use r2r;
pub use ros2_control::*;
pub use ros2_laser_scan::*;
