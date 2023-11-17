#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

mod overwrite;
pub use overwrite::{overwrite, overwrite_str};

mod evaluate;
pub use evaluate::evaluate;
