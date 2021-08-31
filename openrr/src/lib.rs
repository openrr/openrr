#![doc = include_str!("../../README.md")]
#![warn(rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

pub mod apps {
    pub use openrr_apps::*;
}

pub mod client {
    pub use openrr_client::*;
}

pub mod command {
    pub use openrr_command::*;
}

pub mod planner {
    pub use openrr_planner::*;
}

pub mod sleep {
    pub use openrr_sleep::*;
}

pub mod teleop {
    pub use openrr_teleop::*;
}

pub mod config {
    pub use openrr_config::*;
}
