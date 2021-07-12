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

use std::io;

use thiserror::Error;

#[derive(Debug)]
pub enum CollisionPart {
    Start,
    End,
}

/// Error for `openrr_planner`
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("{}", error)]
    Other { error: String },
    #[error("Node name {} not found", .0)]
    NotFound(String),
    #[error("Collision error: {collision_link_names:?} is colliding ({part:?})")]
    Collision {
        part: CollisionPart,
        collision_link_names: Vec<String>,
    },
    #[error("Self Collision error: {collision_link_names:?} is colliding ({part:?})")]
    SelfCollision {
        part: CollisionPart,
        collision_link_names: Vec<(String, String)>,
    },
    #[error("Interpolation error: {:?}", .0)]
    InterpolationError(String),
    #[error("IO error {:?}", source)]
    Io {
        #[from]
        source: io::Error,
    },
    #[error("DoF mismatch {} != {}", .0, .1)]
    DofMismatch(usize, usize),
    #[error("URDF error: {:?}", source)]
    Urdf {
        #[from]
        source: urdf_rs::UrdfError,
    },
    #[error("Path not found {}", .0)]
    PathPlanFail(String),
    #[error("Kinematics error: {:?}", source)]
    KinematicsError {
        #[from]
        source: k::Error,
    },
    #[error("failed to parse {}", .0)]
    ParseError(String),
    #[error("Mesh error {}", .0)]
    MeshError(String),
}

/// Result for `openrr_planner`
pub type Result<T> = ::std::result::Result<T, Error>;
