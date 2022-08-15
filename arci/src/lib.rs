#![doc = include_str!("../README.md")]
#![warn(rust_2018_idioms)]
// buggy: https://github.com/rust-lang/rust-clippy/issues?q=is%3Aissue+derive_partial_eq_without_eq
#![allow(clippy::derive_partial_eq_without_eq)]

mod clients;
mod error;
mod traits;
pub mod utils;
mod waits;

// re-export
pub use async_trait::async_trait;
pub use nalgebra::{self, Isometry2, Isometry3, UnitQuaternion, Vector2, Vector3};

pub use crate::{clients::*, error::*, traits::*, waits::*};
