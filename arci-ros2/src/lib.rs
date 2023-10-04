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
mod node;
mod plugin;
mod ros2_control;
mod ros2_laser_scan;
mod ros2_localization_client;
mod ros2_transform_resolver;
#[allow(missing_docs)]
pub mod utils;

pub use cmd_vel_move_base::*;
pub use navigation::*;
pub use node::*;
// re-export
pub use r2r;
pub use ros2_control::*;
pub use ros2_laser_scan::*;
pub use ros2_localization_client::*;
pub use ros2_transform_resolver::*;
