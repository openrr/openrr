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
mod errors;
pub use errors::Error;

pub mod collision;
pub use collision::{CollisionChecker, FromUrdf};

mod funcs;
pub use funcs::*;

mod ik;
pub use ik::*;

mod planner;
// re-export k::IK modules
pub use k::{InverseKinematicsSolver, JacobianIkSolver};
pub use planner::*;
