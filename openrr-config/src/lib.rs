#![doc = include_str!("../README.md")]
#![warn(missing_docs, rust_2018_idioms)]

mod overwrite;
pub use overwrite::{overwrite, overwrite_str};

mod evaluate;
pub use evaluate::evaluate;
