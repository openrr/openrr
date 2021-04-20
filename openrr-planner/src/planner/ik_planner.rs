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
use k::nalgebra as na;
use na::RealField;
use ncollide3d::shape::Compound;

use super::joint_path_planner::JointPathPlanner;
use crate::errors::*;

/// Joint path planner which supports inverse kinematics
pub struct JointPathPlannerWithIk<T, I>
where
    I: k::InverseKinematicsSolver<T>,
    T: RealField + k::SubsetOf<f64>,
{
    /// Joint Path Planner to be used to find collision free path
    ///
    /// Currently, `JointPathPlanner<N, k::Chain<N>>` is used.
    pub path_planner: JointPathPlanner<T>,
    /// Inverse kinematics solver to find the goal joint angles
    pub ik_solver: I,
}

impl<T, I> JointPathPlannerWithIk<T, I>
where
    //T: RealField + num_traits::Float + k::SubsetOf<f64>,
    T: RealField + k::SubsetOf<f64> + num_traits::Float,
    I: k::InverseKinematicsSolver<T>,
{
    /// Create instance from `JointPathPlannerBuilder` and `InverseKinematicsSolver`
    ///
    /// # Example
    ///
    /// ```
    /// // Create path planner with loading urdf file and set end link name
    /// let planner = openrr_planner::JointPathPlannerBuilder::from_urdf_file("sample.urdf")
    ///     .unwrap()
    ///     .collision_check_margin(0.01)
    ///     .finalize();
    /// // Create inverse kinematics solver
    /// let solver = openrr_planner::JacobianIkSolver::default();
    /// // Create path planner with IK solver
    /// let _planner = openrr_planner::JointPathPlannerWithIk::new(planner, solver);
    /// ```
    pub fn new(path_planner: JointPathPlanner<T>, ik_solver: I) -> Self {
        Self {
            path_planner,
            ik_solver,
        }
    }

    pub fn urdf_robot(&self) -> &Option<urdf_rs::Robot> {
        &self.path_planner.urdf_robot
    }

    /// Just solve IK and do not plan
    pub fn solve_ik(
        &mut self,
        arm: &k::SerialChain<T>,
        target_pose: &na::Isometry3<T>,
    ) -> Result<()> {
        Ok(self.ik_solver.solve(arm, target_pose)?)
    }

    /// Just solve IK with constraints and do not plan
    pub fn solve_ik_with_constraints(
        &mut self,
        arm: &k::SerialChain<T>,
        target_pose: &na::Isometry3<T>,
        c: &k::Constraints,
    ) -> Result<()> {
        Ok(self.ik_solver.solve_with_constraints(arm, target_pose, c)?)
    }

    pub fn colliding_link_names(&self, objects: &Compound<T>) -> Vec<String> {
        self.path_planner.colliding_link_names(objects)
    }

    /// Solve IK and get the path to the final joint positions
    pub fn plan_with_ik(
        &mut self,
        target_name: &str,
        target_pose: &na::Isometry3<T>,
        objects: &Compound<T>,
    ) -> Result<Vec<Vec<T>>> {
        self.plan_with_ik_with_constraints(
            target_name,
            target_pose,
            objects,
            &k::Constraints::default(),
        )
    }

    /// Solve IK with constraints and get the path to the final joint positions
    pub fn plan_with_ik_with_constraints(
        &mut self,
        target_name: &str,
        target_pose: &na::Isometry3<T>,
        objects: &Compound<T>,
        constraints: &k::Constraints,
    ) -> Result<Vec<Vec<T>>> {
        let end_link: &k::Node<T> = self
            .path_planner
            .collision_check_robot
            .find(target_name)
            .ok_or_else(|| Error::NotFound(target_name.to_owned()))?;
        let arm = k::SerialChain::from_end(end_link);
        let initial = arm.joint_positions();
        self.ik_solver
            .solve_with_constraints(&arm, target_pose, constraints)?;
        let goal = arm.joint_positions();
        self.path_planner.plan(&arm, &initial, &goal, objects)
    }

    /// Do not solve IK but get the path to the target joint positions
    pub fn plan_joints<K>(
        &mut self,
        use_joints: &k::Chain<T>,
        start_angles: &[T],
        goal_angles: &[T],
        objects: &Compound<T>,
    ) -> Result<Vec<Vec<T>>> {
        self.path_planner
            .plan(use_joints, start_angles, goal_angles, objects)
    }

    /// Calculate the transforms of all of the links
    pub fn update_transforms(&self) -> Vec<na::Isometry3<T>> {
        self.path_planner.update_transforms()
    }

    /// Get the names of the links
    pub fn joint_names(&self) -> Vec<String> {
        self.path_planner.joint_names()
    }
}
