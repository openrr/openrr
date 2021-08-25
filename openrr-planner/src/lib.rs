/*
Copyright 2017 Takashi Ogura

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
//! # Motion Planning Library for Robotics
//!
//! Get the collision free trajectory of joint angles. `ncollide3d` is used to check the
//! collision between the robot and the environment.
//!

#![warn(rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

mod errors;

pub mod collision;

mod funcs;

mod ik;

mod planner;

// re-export k::IK modules
pub use k::{InverseKinematicsSolver, JacobianIkSolver};

pub use crate::{
    collision::{CollisionDetector, FromUrdf, SelfCollisionChecker, SelfCollisionCheckerConfig},
    errors::Error,
    funcs::*,
    ik::*,
    planner::*,
};
