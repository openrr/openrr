#![doc = include_str!("../README.md")]
#![warn(rust_2018_idioms)]
// buggy: https://github.com/rust-lang/rust-clippy/issues?q=is%3Aissue+derive_partial_eq_without_eq
#![allow(clippy::derive_partial_eq_without_eq)]

mod scoped_sleep;

pub use crate::scoped_sleep::*;
