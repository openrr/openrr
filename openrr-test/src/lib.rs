#![warn(rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

mod dummy_gamepad;
mod dummy_localization;
mod dummy_move_base;
mod dummy_navigation;
mod dummy_speaker;
mod dummy_trajectory_client;
mod dummy_transform_resolver;

pub use crate::{
    dummy_gamepad::*, dummy_localization::*, dummy_move_base::*, dummy_navigation::*,
    dummy_speaker::*, dummy_trajectory_client::*, dummy_transform_resolver::*,
};
