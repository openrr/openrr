#![doc = include_str!("../README.md")]
#![warn(rust_2018_idioms)]
// buggy: https://github.com/rust-lang/rust-clippy/issues?q=is%3Aissue+derive_partial_eq_without_eq
#![allow(clippy::derive_partial_eq_without_eq)]

mod control_mode;
mod control_modes_config;
mod ik;
mod joints;
mod joints_pose_sender;
mod move_base;
mod robot_command_executor;
mod switcher;

pub use crate::{
    control_mode::*, control_modes_config::*, ik::*, joints::*, joints_pose_sender::*,
    move_base::*, robot_command_executor::*, switcher::*,
};
