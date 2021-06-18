//! [`arci`] implementation using ROS1.

#![warn(rust_2018_idioms)]

mod cmd_vel_move_base;
mod error;
mod joy_gamepad;
mod msg;
mod ros_control_action_client;
mod ros_control_client;
mod ros_localization_client;
mod ros_nav_client;
mod ros_robot_client;
mod ros_speak_client;
pub mod ros_transform_resolver;
pub mod rosrust_utils;

// re-export
pub use rosrust::{init, is_ok, rate};

pub use crate::{
    cmd_vel_move_base::*, error::Error, joy_gamepad::*, ros_control_action_client::*,
    ros_control_client::*, ros_localization_client::*, ros_nav_client::*, ros_robot_client::*,
    ros_speak_client::*, ros_transform_resolver::*, rosrust_utils::*,
};
