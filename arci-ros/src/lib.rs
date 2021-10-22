#![doc = include_str!("../README.md")]
#![warn(rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

mod cmd_vel_move_base;
mod error;
mod joy_gamepad;
pub mod msg;
mod msg_utils;
mod ros_control;
mod ros_localization_client;
mod ros_nav_client;
mod ros_robot_client;
mod ros_speak_client;
pub mod ros_transform_resolver;
pub mod rosrust_utils;

// re-export
pub use rosrust::{init, is_ok, name, rate};

pub use crate::{
    cmd_vel_move_base::*, error::Error, joy_gamepad::*, ros_control::*, ros_localization_client::*,
    ros_nav_client::*, ros_robot_client::*, ros_speak_client::*, ros_transform_resolver::*,
    rosrust_utils::*,
};
