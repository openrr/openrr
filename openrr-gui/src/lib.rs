#![doc = include_str!("../README.md")]
#![warn(rust_2018_idioms)]
// buggy: https://github.com/rust-lang/rust-clippy/issues?q=is%3Aissue+derive_partial_eq_without_eq
#![allow(clippy::derive_partial_eq_without_eq)]

mod error;
mod joint_position_sender;
mod velocity_sender;

pub use crate::{error::*, joint_position_sender::*, velocity_sender::*};
