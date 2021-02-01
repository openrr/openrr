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
#![allow(clippy::trivially_copy_pass_by_ref)]

use crate::funcs::*;
use k::nalgebra as na;
use k::{InverseKinematicsSolver, SubsetOf};
use na::RealField;

/// Randomize initial joint angles before solving
#[derive(Debug)]
pub struct RandomInitializeIKSolver<T, I>
where
    I: InverseKinematicsSolver<T>,
    T: RealField,
{
    /// The IK solver to be used after set random joint angles
    pub solver: I,
    /// The number to try to solve
    pub num_max_try: usize,
    phantom: ::std::marker::PhantomData<T>,
}

impl<T, I> RandomInitializeIKSolver<T, I>
where
    T: RealField,
    I: InverseKinematicsSolver<T>,
{
    pub fn new(solver: I, num_max_try: usize) -> Self {
        RandomInitializeIKSolver {
            solver,
            num_max_try,
            phantom: ::std::marker::PhantomData,
        }
    }
}

impl<T, I> InverseKinematicsSolver<T> for RandomInitializeIKSolver<T, I>
where
    T: RealField + SubsetOf<f64>,
    I: InverseKinematicsSolver<T>,
{
    fn solve_with_constraints(
        &self,
        arm: &k::SerialChain<T>,
        target_pose: &na::Isometry3<T>,
        constraints: &k::Constraints,
    ) -> ::std::result::Result<(), k::Error> {
        let mut result = Err(k::Error::NotConvergedError {
            num_tried: 0,
            position_diff: na::Vector3::new(0.0, 0.0, 0.0),
            rotation_diff: na::Vector3::new(0.0, 0.0, 0.0),
        });
        let limits = arm.iter_joints().map(|j| j.limits).collect();
        let initial_angles = arm.joint_positions();

        for _ in 0..self.num_max_try {
            result = self
                .solver
                .solve_with_constraints(arm, target_pose, constraints);
            if result.is_ok() {
                return result;
            }
            let mut new_angles = generate_random_joint_positions_from_limits(&limits);
            modify_to_nearest_angle(&initial_angles, &mut new_angles, &limits);
            arm.set_joint_positions(&new_angles)?;
        }
        // failed
        arm.set_joint_positions(&initial_angles)?;
        result
    }
}

/// Check the poses which can be reached by the robot arm
pub fn get_reachable_region<T, I>(
    ik_solver: &I,
    arm: &k::SerialChain<T>,
    initial_pose: &na::Isometry3<T>,
    constraints: &k::Constraints,
    max_point: na::Vector3<T>,
    min_point: na::Vector3<T>,
    unit_check_length: T,
) -> Vec<na::Isometry3<T>>
where
    T: RealField + k::SubsetOf<f64>,
    I: InverseKinematicsSolver<T>,
{
    let initial_angles = arm.joint_positions();
    let mut z = min_point[2];
    let mut solved_poses = Vec::new();
    let mut target_pose = *initial_pose;
    while z < max_point[2] {
        target_pose.translation.vector[2] = z;
        let mut y = min_point[1];
        while y < max_point[1] {
            target_pose.translation.vector[1] = y;
            let mut x = min_point[0];
            while x < max_point[0] {
                //
                target_pose.translation.vector[0] = x;
                arm.set_joint_positions_unchecked(&initial_angles);
                if ik_solver
                    .solve_with_constraints(arm, &target_pose, &constraints)
                    .is_ok()
                {
                    solved_poses.push(target_pose);
                }
                x += unit_check_length;
            }
            y += unit_check_length;
        }
        z += unit_check_length;
    }
    solved_poses
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn get_region() {
        let chain = k::Chain::<f32>::from_urdf_file("sample.urdf").unwrap();

        // Set initial joint angles
        let angles = vec![0.2, 0.2, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0];

        chain.set_joint_positions(&angles).unwrap();
        println!("initial angles={:?}", chain.joint_positions());

        let target_link = chain.find("l_wrist_pitch").unwrap();

        // Get the transform of the end of the manipulator (forward kinematics)
        chain.update_transforms();
        let target = target_link.world_transform().unwrap();
        println!("{:?}", target.translation);
        // Create IK solver with default settings
        let solver = k::JacobianIkSolver::default();

        // Create a set of joints from end joint
        let arm = k::SerialChain::from_end(target_link);
        let regions = get_reachable_region(
            &solver,
            &arm,
            &target,
            &k::Constraints::default(),
            na::Vector3::new(0.8, 0.9, 0.9),
            na::Vector3::new(0.0, -0.9, 0.0),
            0.1,
        );
        assert_eq!(regions.len(), 172);
    }
}
