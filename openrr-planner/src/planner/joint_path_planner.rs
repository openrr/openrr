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
use std::path::Path;

use k::nalgebra as na;
use na::RealField;
use ncollide3d::shape::Compound;
use tracing::*;

use crate::{
    collision::{CollisionDetector, RobotCollisionDetector},
    errors::*,
    funcs::*,
};

/// Collision Avoidance Path Planner
pub struct JointPathPlanner<N>
where
    N: RealField + Copy + k::SubsetOf<f64>,
{
    /// Robot collision detector
    pub robot_collision_detector: RobotCollisionDetector<N>,
    /// Unit length for searching
    ///
    /// If the value is large, the path become sparse.
    pub step_length: N,
    /// Max num of RRT search loop
    pub max_try: usize,
    /// Num of path smoothing trials
    pub num_smoothing: usize,
}

impl<N> JointPathPlanner<N>
where
    N: RealField + num_traits::Float + k::SubsetOf<f64>,
{
    /// Create `JointPathPlanner`
    pub fn new(
        robot_collision_detector: RobotCollisionDetector<N>,
        step_length: N,
        max_try: usize,
        num_smoothing: usize,
    ) -> Self {
        JointPathPlanner {
            robot_collision_detector,
            step_length,
            max_try,
            num_smoothing,
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
            Ok(()) => !self.robot_collision_detector.is_collision_detected(objects),
            Err(err) => {
                debug!("is_feasible: {}", err);
                false
            }
        }
    }

    /// Check if the joint_positions are OK
    pub fn is_feasible_with_self(&self, using_joints: &k::Chain<N>, joint_positions: &[N]) -> bool {
        match using_joints.set_joint_positions(joint_positions) {
            Ok(()) => !self.robot_collision_detector.is_self_collision_detected(),
            Err(err) => {
                debug!("is_feasible: {}", err);
                false
            }
        }
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
            let collision_link_names = self
                .robot_collision_detector
                .env_collision_link_names(objects);
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::Collision {
                point: UnfeasibleTrajectory::StartPoint,
                collision_link_names,
            });
        } else if !self.is_feasible(using_joints, goal_angles, objects) {
            let collision_link_names = self
                .robot_collision_detector
                .env_collision_link_names(objects);
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::Collision {
                point: UnfeasibleTrajectory::GoalPoint,
                collision_link_names,
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
            let collision_link_names = self.robot_collision_detector.self_collision_link_pairs();
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::SelfCollision {
                point: UnfeasibleTrajectory::StartPoint,
                collision_link_names,
            });
        } else if !self.is_feasible_with_self(using_joints, goal_angles) {
            let collision_link_names = self.robot_collision_detector.self_collision_link_pairs();
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::SelfCollision {
                point: UnfeasibleTrajectory::GoalPoint,
                collision_link_names,
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
        self.robot_collision_detector.robot.update_transforms()
    }

    /// Get the names of the links
    pub fn joint_names(&self) -> Vec<String> {
        self.robot_collision_detector
            .robot
            .iter_joints()
            .map(|j| j.name.clone())
            .collect()
    }
}

/// Builder pattern to create `JointPathPlanner`
pub struct JointPathPlannerBuilder<N>
where
    N: RealField + Copy + k::SubsetOf<f64>,
{
    robot_collision_detector: RobotCollisionDetector<N>,
    step_length: N,
    max_try: usize,
    num_smoothing: usize,
    collision_check_margin: Option<N>,
    self_collision_pairs: Vec<(String, String)>,
}

impl<N> JointPathPlannerBuilder<N>
where
    N: RealField + num_traits::Float + k::SubsetOf<f64>,
{
    /// Create from components
    ///
    /// There are also some utility functions to create from urdf
    pub fn new(robot_collision_detector: RobotCollisionDetector<N>) -> Self {
        JointPathPlannerBuilder {
            robot_collision_detector,
            step_length: na::convert(0.1),
            max_try: 5000,
            num_smoothing: 100,
            collision_check_margin: None,
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
            self.robot_collision_detector.collision_detector.prediction = margin;
        }
        let mut planner = JointPathPlanner::new(
            self.robot_collision_detector,
            self.step_length,
            self.max_try,
            self.num_smoothing,
        );
        planner.robot_collision_detector.self_collision_pairs = self.self_collision_pairs;
        planner
    }

    /// Try to create `JointPathPlannerBuilder` instance from URDF file and end link name
    pub fn from_urdf_file<P>(file: P) -> Result<JointPathPlannerBuilder<N>>
    where
        P: AsRef<Path>,
    {
        let urdf_robot = urdf_rs::utils::read_urdf_or_xacro(file.as_ref())?;
        Ok(JointPathPlannerBuilder::from_urdf_robot(urdf_robot))
    }

    /// Try to create `JointPathPlannerBuilder` instance from `urdf_rs::Robot` instance
    pub fn from_urdf_robot(urdf_robot: urdf_rs::Robot) -> JointPathPlannerBuilder<N> {
        let robot = k::Chain::from(&urdf_robot);
        let default_margin = na::convert(0.0);
        let collision_detector = CollisionDetector::from_urdf_robot(&urdf_robot, default_margin);
        let robot_collision_detector =
            RobotCollisionDetector::new(robot, collision_detector, vec![]);
        JointPathPlannerBuilder::new(robot_collision_detector)
    }
}

#[cfg(test)]
mod tests {
    use na::{Isometry3, Vector3};
    use ncollide3d::shape::Cuboid;

    use super::*;

    #[test]
    fn collision_check() {
        let urdf_robot = urdf_rs::read_file("sample.urdf").unwrap();
        let detector = CollisionDetector::from_urdf_robot(&urdf_robot, 0.01);

        let target = Cuboid::new(Vector3::new(0.5, 1.0, 0.5));
        let target_pose = Isometry3::new(Vector3::new(0.9, 0.0, 0.0), na::zero());

        let robot = k::Chain::<f32>::from(&urdf_robot);

        let names: Vec<String> = detector.detect_env(&robot, &target, &target_pose).collect();
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
        let names: Vec<String> = detector.detect_env(&robot, &target, &target_pose).collect();
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
        let names: Vec<String> = detector.detect_env(&robot, &target, &target_pose).collect();
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
