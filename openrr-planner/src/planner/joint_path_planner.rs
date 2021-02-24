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
use crate::collision::CollisionChecker;
use crate::errors::*;
use crate::funcs::*;
use k::nalgebra as na;
use na::RealField;
use ncollide3d::shape::Compound;
use std::path::Path;
use tracing::*;

/// Collision Avoidance Path Planner
pub struct JointPathPlanner<N>
where
    N: RealField + k::SubsetOf<f64>,
{
    /// Instance of `k::HasLinks` to check the collision
    pub collision_check_robot: k::Chain<N>,
    /// Collision checker
    pub collision_checker: CollisionChecker<N>,
    /// Unit length for searching
    ///
    /// If the value is large, the path become sparse.
    pub step_length: N,
    /// Max num of RRT search loop
    pub max_try: usize,
    /// Num of path smoothing trials
    pub num_smoothing: usize,
    /// The robot instance which is used to create the robot model
    pub urdf_robot: Option<urdf_rs::Robot>,
    /// Optional self collision check node names
    pub self_collision_pairs: Vec<(String, String)>,
}

impl<N> JointPathPlanner<N>
where
    N: RealField + num_traits::Float + k::SubsetOf<f64>,
{
    /// Create `JointPathPlanner`
    pub fn new(
        collision_check_robot: k::Chain<N>,
        collision_checker: CollisionChecker<N>,
        step_length: N,
        max_try: usize,
        num_smoothing: usize,
    ) -> Self {
        JointPathPlanner {
            collision_check_robot,
            collision_checker,
            step_length,
            max_try,
            num_smoothing,
            urdf_robot: None,
            self_collision_pairs: vec![],
        }
    }
    /// Check if the joint_positions are OK
    pub fn is_feasible(
        &self,
        using_joints: &k::Chain<N>,
        joint_positions: &[N],
        objects: &Compound<N>,
    ) -> bool {
        match using_joints.set_joint_positions(joint_positions) {
            Ok(()) => !self.has_any_colliding(objects),
            Err(err) => {
                debug!("is_feasible: {}", err);
                false
            }
        }
    }
    /// Check if there are any colliding links
    pub fn has_any_colliding(&self, objects: &Compound<N>) -> bool {
        for shape in objects.shapes() {
            if self
                .collision_checker
                .check_env(&self.collision_check_robot, &*shape.1, &shape.0)
                .next()
                .is_some()
            {
                return true;
            }
        }
        false
    }
    /// Get the names of colliding links
    pub fn colliding_link_names(&self, objects: &Compound<N>) -> Vec<String> {
        let mut ret = Vec::new();
        for shape in objects.shapes() {
            let mut colliding_names = self
                .collision_checker
                .check_env(&self.collision_check_robot, &*shape.1, &shape.0)
                .collect();
            ret.append(&mut colliding_names);
        }
        ret
    }

    /// Check if the joint_positions are OK
    pub fn is_feasible_with_self(&self, using_joints: &k::Chain<N>, joint_positions: &[N]) -> bool {
        match using_joints.set_joint_positions(joint_positions) {
            Ok(()) => !self.has_any_colliding_with_self(),
            Err(err) => {
                debug!("is_feasible: {}", err);
                false
            }
        }
    }
    /// Check if there are any colliding links
    pub fn has_any_colliding_with_self(&self) -> bool {
        self.collision_checker
            .check_self(&self.collision_check_robot, &self.self_collision_pairs)
            .next()
            .is_some()
    }
    /// Get the names of colliding links
    pub fn colliding_link_names_with_self(&self) -> Vec<(String, String)> {
        self.collision_checker
            .check_self(&self.collision_check_robot, &self.self_collision_pairs)
            .collect()
    }

    /// Plan the sequence of joint angles of `using_joints`
    ///
    /// # Arguments
    ///
    /// - `using_joints`: part of collision_check_robot. the dof of the following angles must be same as this model.
    /// - `start_angles`: initial joint angles of `using_joints`.
    /// - `goal_angles`: goal joint angles of `using_joints`.
    /// - `objects`: The collision between `self.collision_check_robot` and `objects` will be checked.
    pub fn plan(
        &self,
        using_joints: &k::Chain<N>,
        start_angles: &[N],
        goal_angles: &[N],
        objects: &Compound<N>,
    ) -> Result<Vec<Vec<N>>> {
        let limits = using_joints.iter_joints().map(|j| j.limits).collect();
        let step_length = self.step_length;
        let max_try = self.max_try;
        let current_angles = using_joints.joint_positions();
        if !self.is_feasible(using_joints, start_angles, objects) {
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::Collision {
                part: CollisionPart::Start,
                collision_link_names: self.colliding_link_names(objects),
            });
        } else if !self.is_feasible(using_joints, goal_angles, objects) {
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::Collision {
                part: CollisionPart::End,
                collision_link_names: self.colliding_link_names(objects),
            });
        }
        let mut path = match rrt::dual_rrt_connect(
            start_angles,
            goal_angles,
            |angles: &[N]| self.is_feasible(using_joints, angles, objects),
            || generate_random_joint_positions_from_limits(&limits),
            step_length,
            max_try,
        ) {
            Ok(p) => p,
            Err(error) => {
                using_joints.set_joint_positions(&current_angles)?;
                return Err(Error::PathPlanFail(error));
            }
        };
        let num_smoothing = self.num_smoothing;
        rrt::smooth_path(
            &mut path,
            |angles: &[N]| self.is_feasible(using_joints, angles, objects),
            step_length,
            num_smoothing,
        );
        Ok(path)
    }
    /// Plan the sequence of joint angles of `using_joints` to avoid self collision.
    ///
    /// # Arguments
    ///
    /// - `using_joints`: part of collision_check_robot. the dof of the following angles must be same as this model.
    /// - `start_angles`: initial joint angles of `using_joints`.
    /// - `goal_angles`: goal joint angles of `using_joints`.
    pub fn plan_avoid_self_collision(
        &self,
        using_joints: &k::Chain<N>,
        start_angles: &[N],
        goal_angles: &[N],
    ) -> Result<Vec<Vec<N>>> {
        let limits = using_joints.iter_joints().map(|j| j.limits).collect();
        let step_length = self.step_length;
        let max_try = self.max_try;
        let current_angles = using_joints.joint_positions();
        if !self.is_feasible_with_self(using_joints, start_angles) {
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::SelfCollision {
                part: CollisionPart::Start,
                collision_link_names: self.colliding_link_names_with_self(),
            });
        } else if !self.is_feasible_with_self(using_joints, goal_angles) {
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::SelfCollision {
                part: CollisionPart::End,
                collision_link_names: self.colliding_link_names_with_self(),
            });
        }
        let mut path = match rrt::dual_rrt_connect(
            start_angles,
            goal_angles,
            |angles: &[N]| self.is_feasible_with_self(using_joints, angles),
            || generate_random_joint_positions_from_limits(&limits),
            step_length,
            max_try,
        ) {
            Ok(p) => p,
            Err(error) => {
                using_joints.set_joint_positions(&current_angles)?;
                return Err(Error::PathPlanFail(error));
            }
        };
        let num_smoothing = self.num_smoothing;
        rrt::smooth_path(
            &mut path,
            |angles: &[N]| self.is_feasible_with_self(using_joints, angles),
            step_length,
            num_smoothing,
        );
        Ok(path)
    }
    /// Calculate the transforms of all of the links
    pub fn update_transforms(&self) -> Vec<na::Isometry3<N>> {
        self.collision_check_robot.update_transforms()
    }

    /// Get the names of the links
    pub fn joint_names(&self) -> Vec<String> {
        self.collision_check_robot
            .iter_joints()
            .map(|j| j.name.clone())
            .collect()
    }
}

/// Builder pattern to create `JointPathPlanner`
pub struct JointPathPlannerBuilder<N>
where
    N: RealField,
{
    collision_check_robot: k::Chain<N>,
    collision_checker: CollisionChecker<N>,
    step_length: N,
    max_try: usize,
    num_smoothing: usize,
    collision_check_margin: Option<N>,
    urdf_robot: Option<urdf_rs::Robot>,
    self_collision_pairs: Vec<(String, String)>,
}

impl<N> JointPathPlannerBuilder<N>
where
    N: RealField + num_traits::Float + k::SubsetOf<f64>,
{
    /// Create from components
    ///
    /// There are also some utility functions to create from urdf
    pub fn new(urdf_robot: urdf_rs::Robot, collision_checker: CollisionChecker<N>) -> Self {
        let collision_check_robot = (&urdf_robot).into();
        let urdf_robot = Some(urdf_robot);
        JointPathPlannerBuilder {
            collision_check_robot,
            collision_checker,
            step_length: na::convert(0.1),
            max_try: 5000,
            num_smoothing: 100,
            collision_check_margin: None,
            urdf_robot,
            self_collision_pairs: vec![],
        }
    }
    pub fn collision_check_margin(mut self, length: N) -> Self {
        self.collision_check_margin = Some(length);
        self
    }
    pub fn step_length(mut self, step_length: N) -> Self {
        self.step_length = step_length;
        self
    }
    pub fn max_try(mut self, max_try: usize) -> Self {
        self.max_try = max_try;
        self
    }
    pub fn num_smoothing(mut self, num_smoothing: usize) -> Self {
        self.num_smoothing = num_smoothing;
        self
    }
    pub fn self_collision_pairs(mut self, self_collision_pairs: Vec<(String, String)>) -> Self {
        self.self_collision_pairs = self_collision_pairs;
        self
    }
    pub fn finalize(mut self) -> JointPathPlanner<N> {
        if let Some(margin) = self.collision_check_margin {
            self.collision_checker.prediction = margin;
        }
        let mut planner = JointPathPlanner::new(
            self.collision_check_robot,
            self.collision_checker,
            self.step_length,
            self.max_try,
            self.num_smoothing,
        );
        planner.urdf_robot = self.urdf_robot;
        planner.self_collision_pairs = self.self_collision_pairs;
        planner
    }
}

impl<N> JointPathPlannerBuilder<N>
where
    N: RealField + k::SubsetOf<f64> + num_traits::Float,
{
    /// Try to create `JointPathPlannerBuilder` instance from URDF file and end link name
    pub fn from_urdf_file<P>(file: P) -> Result<JointPathPlannerBuilder<N>>
    where
        P: AsRef<Path>,
    {
        let robot = urdf_rs::utils::read_urdf_or_xacro(file.as_ref())?;
        let default_margin = na::convert(0.0);
        let collision_checker = CollisionChecker::from_urdf_robot_with_base_dir(
            &robot,
            file.as_ref().parent(),
            default_margin,
        );
        Ok(get_joint_path_planner_builder_from_urdf(
            robot,
            collision_checker,
        ))
    }
    /// Try to create `JointPathPlannerBuilder` instance from `urdf_rs::Robot` instance
    pub fn from_urdf_robot<P>(robot: urdf_rs::Robot) -> JointPathPlannerBuilder<N> {
        let default_margin = na::convert(0.0);
        let collision_checker = CollisionChecker::from_urdf_robot(&robot, default_margin);
        get_joint_path_planner_builder_from_urdf(robot, collision_checker)
    }
}

fn get_joint_path_planner_builder_from_urdf<N>(
    urdf_robot: urdf_rs::Robot,
    collision_checker: CollisionChecker<N>,
) -> JointPathPlannerBuilder<N>
where
    N: RealField + k::SubsetOf<f64> + num_traits::Float,
{
    JointPathPlannerBuilder::new(urdf_robot, collision_checker)
}

#[cfg(test)]
mod tests {
    use super::*;
    use na::{Isometry3, Vector3};
    use ncollide3d::shape::Cuboid;

    #[test]
    fn collision_check() {
        let urdf_robot = urdf_rs::read_file("sample.urdf").unwrap();
        let checker = CollisionChecker::from_urdf_robot(&urdf_robot, 0.01);

        let target = Cuboid::new(Vector3::new(0.5, 1.0, 0.5));
        let target_pose = Isometry3::new(Vector3::new(0.9, 0.0, 0.0), na::zero());

        let robot = k::Chain::<f32>::from(&urdf_robot);

        let names: Vec<String> = checker.check_env(&robot, &target, &target_pose).collect();
        assert_eq!(
            names,
            vec![
                "l_elbow_pitch",
                "l_wrist_yaw",
                "l_wrist_pitch",
                "l_gripper_linear2",
                "l_gripper_linear1",
            ]
        );
        let angles = vec![-1.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        robot.set_joint_positions(&angles).unwrap();
        let names: Vec<String> = checker.check_env(&robot, &target, &target_pose).collect();
        assert_eq!(
            names,
            vec![
                "l_wrist_yaw",
                "l_wrist_pitch",
                "l_gripper_linear2",
                "l_gripper_linear1"
            ]
        );
        let target_pose = Isometry3::new(Vector3::new(0.7, 0.0, 0.0), na::zero());
        let names: Vec<String> = checker.check_env(&robot, &target, &target_pose).collect();
        assert_eq!(
            names,
            vec![
                "root",
                "l_shoulder_roll",
                "l_elbow_pitch",
                "l_wrist_yaw",
                "l_wrist_pitch",
                "l_gripper_linear2",
                "l_gripper_linear1",
            ]
        );
    }
    #[test]
    fn from_urdf() {
        let _planner = JointPathPlannerBuilder::from_urdf_file("sample.urdf")
            .unwrap()
            .collision_check_margin(0.01)
            .finalize();
    }
}
