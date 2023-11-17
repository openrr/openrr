#![doc = include_str!("../README.md")]

mod error;
mod robot_command;

pub use crate::{error::*, robot_command::*};
