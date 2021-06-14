//! Abstract Robot Control Interface.

#![warn(rust_2018_idioms)]

mod clients;
mod error;
mod traits;
pub mod utils;
mod waits;

// re-export
pub use async_trait::async_trait;
pub use nalgebra::{Isometry2, Isometry3, UnitQuaternion, Vector2, Vector3};

pub use crate::{clients::*, error::*, traits::*, waits::*};
