#![doc = include_str!("../README.md")]
#![warn(rust_2018_idioms)]
// buggy: https://github.com/rust-lang/rust-clippy/issues?q=is%3Aissue+derive_partial_eq_without_eq
#![allow(clippy::derive_partial_eq_without_eq)]

mod error;
mod robot_config;
mod robot_teleop_config;
pub mod utils;

pub use crate::{error::*, robot_config::*, robot_teleop_config::*};
