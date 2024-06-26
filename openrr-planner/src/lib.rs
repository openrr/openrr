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

#![doc = include_str!("../README.md")]
#![allow(missing_debug_implementations)] // TODO: Some ncollide3d types don't implement Debug
#![allow(clippy::needless_doctest_main)]

mod errors;

pub mod collision;

mod funcs;

mod ik;

mod planner;

// re-export k::IK modules
pub use k::{InverseKinematicsSolver, JacobianIkSolver};

pub use crate::{
    collision::{CollisionDetector, FromUrdf, SelfCollisionChecker, SelfCollisionCheckerConfig},
    errors::{Error, Result},
    funcs::*,
    ik::*,
    planner::*,
};
