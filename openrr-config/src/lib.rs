#![doc = include_str!("../README.md")]
#![warn(missing_docs, rust_2018_idioms)]
// buggy: https://github.com/rust-lang/rust-clippy/issues?q=is%3Aissue+derive_partial_eq_without_eq
#![allow(clippy::derive_partial_eq_without_eq)]

mod overwrite;
pub use overwrite::{overwrite, overwrite_str};

mod evaluate;
pub use evaluate::evaluate;
