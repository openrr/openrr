#![doc = include_str!("../../README.md")]
#![warn(rust_2018_idioms)]
// buggy: https://github.com/rust-lang/rust-clippy/issues?q=is%3Aissue+derive_partial_eq_without_eq
#![allow(clippy::derive_partial_eq_without_eq)]

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

pub mod teleop {
    pub use openrr_teleop::*;
}

pub mod config {
    pub use openrr_config::*;
}
