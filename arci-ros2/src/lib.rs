#![doc = include_str!("../README.md")]
#![cfg(feature = "ros2")]
#![warn(future_incompatible, missing_docs)]
#![allow(missing_debug_implementations)] // TODO: Some r2r types don't implement Debug

mod cmd_vel_move_base;
mod motor_drive;
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
pub use motor_drive::*;
pub use navigation::*;
pub use node::*;
// re-export
pub use r2r;
pub use ros2_control::*;
pub use ros2_laser_scan::*;
pub use ros2_localization_client::*;
pub use ros2_transform_resolver::*;
