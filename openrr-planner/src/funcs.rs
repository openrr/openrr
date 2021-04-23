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
#![allow(clippy::trivially_copy_pass_by_ref, clippy::ptr_arg)]

use std::f64::consts::PI;

use k::{nalgebra as na, nalgebra::RealField};
use num_traits::Float;
use trajectory::{CubicSpline, Trajectory};

use crate::errors::*;

type Limits<T> = Vec<Option<k::joint::Range<T>>>;

/// Clamp joint angles to set angles safely
pub fn generate_clamped_joint_positions_from_limits<T>(
    angles: &[T],
    limits: &Limits<T>,
) -> Result<Vec<T>>
where
    T: RealField,
{
    if angles.len() != limits.len() {
        return Err(Error::DofMismatch(angles.len(), limits.len()));
    }
    Ok(limits
        .iter()
        .zip(angles.iter())
        .map(|(range, angle)| match *range {
            Some(ref range) => {
                if *angle > range.max {
                    range.max
                } else if *angle < range.min {
                    range.min
                } else {
                    *angle
                }
            }
            None => *angle,
        })
        .collect())
}

#[deprecated(
    since = "0.7.0",
    note = "Please use k::Chain::set_joint_positions_clamped"
)]
/// Set joint positions safely
///
/// The input vec is clamped to the limits.
pub fn set_clamped_joint_positions<T>(chain: &k::Chain<T>, vec: &[T]) -> Result<()>
where
    T: RealField + k::SubsetOf<f64>,
{
    let limits = chain.iter_joints().map(|j| j.limits).collect::<Vec<_>>();
    let clamped = generate_clamped_joint_positions_from_limits(vec, &limits)?;
    chain.set_joint_positions(&clamped)?;
    Ok(())
}

/// Generate random joint angles from the optional limits
///
/// If the limit is None, -PI <-> PI is used.
pub fn generate_random_joint_positions_from_limits<T>(limits: &Limits<T>) -> Vec<T>
where
    T: RealField,
{
    limits
        .iter()
        .map(|range| match *range {
            Some(ref range) => (range.max - range.min) * na::convert(rand::random()) + range.min,
            None => na::convert::<f64, T>(rand::random::<f64>() - 0.5) * na::convert(2.0 * PI),
        })
        .collect()
}

/// If the joint has no limit, select the nearest value from (x + 2pi *).
///
/// ```
/// let mut a = vec![0.1f64, 10.0];
/// let limits = vec![Some(k::joint::Range::new(0.0, 0.2)), None];
/// openrr_planner::modify_to_nearest_angle(&vec![1.0, 0.5], &mut a, &limits);
/// assert_eq!(a[0], 0.1, "no change");
/// assert!((a[1] - 3.716814).abs() < 0.000001);
/// ```
pub fn modify_to_nearest_angle<T>(vec1: &[T], vec2: &mut [T], limits: &Limits<T>)
where
    T: RealField,
{
    assert_eq!(vec1.len(), vec2.len());
    for i in 0..vec1.len() {
        if limits[i].is_none() {
            // TODO: deal not only no limit
            let pi2 = T::pi() * na::convert(2.0);
            let dist1 = (vec1[i] - vec2[i]).abs();
            let dist2 = (vec1[i] - (vec2[i] - pi2)).abs();
            if dist1 > dist2 {
                vec2[i] -= pi2;
            } else {
                let dist3 = (vec1[i] - (vec2[i] + pi2)).abs();
                if dist1 > dist3 {
                    vec2[i] += pi2;
                }
            }
        }
    }
}

/// Struct for a point of a trajectory with multiple dimensions.
#[derive(Debug, Clone)]
pub struct TrajectoryPoint<T> {
    pub position: Vec<T>,
    pub velocity: Vec<T>,
    pub acceleration: Vec<T>,
}

impl<T> TrajectoryPoint<T> {
    /// Create trajectory point
    pub fn new(position: Vec<T>, velocity: Vec<T>, acceleration: Vec<T>) -> Self {
        Self {
            position,
            velocity,
            acceleration,
        }
    }
}

/// Interpolate position vectors
///
/// returns vector of (position, velocity, acceleration)
///
/// # Example
///
/// ```
/// let points = openrr_planner::interpolate(&[vec![0.0, 1.0], vec![2.0, 0.0]], 1.0, 0.1).unwrap();
/// assert_eq!(points.len(), 12);
/// assert_eq!(points[0].position[0], 0.0);
/// assert_eq!(points[0].position[1], 1.0);
/// assert_eq!(points[1].position[0], 0.2);
/// assert_eq!(points[1].position[1], 0.9);
/// ```
pub fn interpolate<T>(
    points: &[Vec<T>],
    total_duration: T,
    unit_duration: T,
) -> Option<Vec<TrajectoryPoint<T>>>
where
    T: Float,
{
    let key_frame_unit_duration = total_duration / (T::from(points.len())? - T::one());
    let times = (0_usize..points.len())
        .map(|i| T::from(i).unwrap() * key_frame_unit_duration)
        .collect::<Vec<T>>();
    assert_eq!(times.len(), points.len());

    let spline = CubicSpline::new(times, points.to_vec())?;
    let mut t = T::zero();
    let mut ret = Vec::with_capacity(points.len());
    while t < total_duration {
        ret.push(TrajectoryPoint {
            position: spline.position(t)?,
            velocity: spline.velocity(t)?,
            acceleration: spline.acceleration(t)?,
        });
        t = t + unit_duration;
    }
    // Add final point
    ret.push(TrajectoryPoint {
        position: spline.position(total_duration)?,
        velocity: spline.velocity(total_duration)?,
        acceleration: spline.acceleration(total_duration)?,
    });
    Some(ret)
}

/// Set random joint angles
pub fn set_random_joint_positions<T>(robot: &k::Chain<T>) -> ::std::result::Result<(), k::Error>
where
    T: RealField + k::SubsetOf<f64>,
{
    let limits = robot.iter_joints().map(|j| j.limits).collect();
    robot.set_joint_positions(&generate_random_joint_positions_from_limits(&limits))
}
