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
use std::{path::Path, sync::Arc};

use k::nalgebra as na;
use na::RealField;
use ncollide3d::shape::Compound;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
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
    /// Robot for reference (read only and assumed to hold the latest full states)
    reference_robot: Arc<k::Chain<N>>,
    /// Robot collision detector
    robot_collision_detector: RobotCollisionDetector<N>,
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
        reference_robot: Arc<k::Chain<N>>,
        robot_collision_detector: RobotCollisionDetector<N>,
        step_length: N,
        max_try: usize,
        num_smoothing: usize,
    ) -> Self {
        JointPathPlanner {
            reference_robot,
            robot_collision_detector,
            step_length,
            max_try,
            num_smoothing,
        }
    }

    /// Check if the joint_positions are OK
    fn is_feasible(
        &self,
        using_joints: &k::Chain<N>,
        joint_positions: &[N],
        objects: &Compound<N>,
    ) -> bool {
        match using_joints.set_joint_positions(joint_positions) {
            Ok(()) => !self.robot_collision_detector.is_collision_detected(objects),
            Err(err) => {
                debug!("is_feasible: {err}");
                false
            }
        }
    }

    /// Check if the joint_positions are OK
    fn is_feasible_with_self(&self, using_joints: &k::Chain<N>, joint_positions: &[N]) -> bool {
        match using_joints.set_joint_positions(joint_positions) {
            Ok(()) => !self.robot_collision_detector.is_self_collision_detected(),
            Err(err) => {
                debug!("is_feasible: {err}");
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
        using_joint_names: &[String],
        start_angles: &[N],
        goal_angles: &[N],
        objects: &Compound<N>,
    ) -> Result<Vec<Vec<N>>> {
        self.sync_joint_positions_with_reference();

        let using_joints =
            create_chain_from_joint_names(self.collision_check_robot(), using_joint_names)?;
        let limits = using_joints.iter_joints().map(|j| j.limits).collect();
        let step_length = self.step_length;
        let max_try = self.max_try;
        let current_angles = using_joints.joint_positions();

        if !self.is_feasible(&using_joints, start_angles, objects) {
            let collision_link_names = self.env_collision_link_names(objects);
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::Collision {
                point: UnfeasibleTrajectoryPoint::Start,
                collision_link_names,
            });
        } else if !self.is_feasible(&using_joints, goal_angles, objects) {
            let collision_link_names = self.env_collision_link_names(objects);
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::Collision {
                point: UnfeasibleTrajectoryPoint::Goal,
                collision_link_names,
            });
        }

        let mut path = match rrt::dual_rrt_connect(
            start_angles,
            goal_angles,
            |angles: &[N]| self.is_feasible(&using_joints, angles, objects),
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
            |angles: &[N]| self.is_feasible(&using_joints, angles, objects),
            step_length,
            num_smoothing,
        );

        // The joint positions of using_joint can be changed in the smoothing,
        // so we need to surely set the goal at the end.
        using_joints.set_joint_positions(goal_angles)?;
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
        using_joint_names: &[String],
        start_angles: &[N],
        goal_angles: &[N],
    ) -> Result<Vec<Vec<N>>> {
        self.sync_joint_positions_with_reference();

        let using_joints =
            create_chain_from_joint_names(self.collision_check_robot(), using_joint_names)?;
        let limits = using_joints.iter_joints().map(|j| j.limits).collect();
        let step_length = self.step_length;
        let max_try = self.max_try;
        let current_angles = using_joints.joint_positions();

        if !self.is_feasible_with_self(&using_joints, start_angles) {
            let collision_link_names = self.self_collision_link_pairs();
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::SelfCollision {
                point: UnfeasibleTrajectoryPoint::Start,
                collision_link_names,
            });
        } else if !self.is_feasible_with_self(&using_joints, goal_angles) {
            let collision_link_names = self.self_collision_link_pairs();
            using_joints.set_joint_positions(&current_angles)?;
            return Err(Error::SelfCollision {
                point: UnfeasibleTrajectoryPoint::Goal,
                collision_link_names,
            });
        }

        let mut path = match rrt::dual_rrt_connect(
            start_angles,
            goal_angles,
            |angles: &[N]| self.is_feasible_with_self(&using_joints, angles),
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
            |angles: &[N]| self.is_feasible_with_self(&using_joints, angles),
            step_length,
            num_smoothing,
        );
        Ok(path)
    }

    /// Synchronize joint positions of the planning robot model with the reference robot
    pub fn sync_joint_positions_with_reference(&self) {
        self.collision_check_robot()
            .set_joint_positions_clamped(self.reference_robot.joint_positions().as_slice());
    }

    /// Calculate the transforms of all of the links
    pub fn update_transforms(&self) -> Vec<na::Isometry3<N>> {
        self.collision_check_robot().update_transforms()
    }

    /// Get the names of the links
    pub fn joint_names(&self) -> Vec<String> {
        self.collision_check_robot()
            .iter_joints()
            .map(|j| j.name.clone())
            .collect()
    }

    // Get the robot model used for collision checking
    pub fn collision_check_robot(&self) -> &k::Chain<N> {
        &self.robot_collision_detector.robot
    }

    /// Get names of links colliding with environmental objects
    /// objects: environmental objects
    pub fn env_collision_link_names(&self, objects: &Compound<N>) -> Vec<String> {
        self.robot_collision_detector
            .env_collision_link_names(objects)
    }

    /// Get names of self-colliding links
    pub fn self_collision_link_pairs(&self) -> Vec<(String, String)> {
        self.robot_collision_detector.self_collision_link_pairs()
    }
}

/// Builder pattern to create `JointPathPlanner`
pub struct JointPathPlannerBuilder<N>
where
    N: RealField + Copy + k::SubsetOf<f64>,
{
    reference_robot: Option<Arc<k::Chain<N>>>,
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
            reference_robot: None,
            robot_collision_detector,
            step_length: na::convert(0.1),
            max_try: 5000,
            num_smoothing: 100,
            collision_check_margin: None,
            self_collision_pairs: vec![],
        }
    }

    pub fn reference_robot(mut self, robot: Arc<k::Chain<N>>) -> Self {
        self.reference_robot = Some(robot);
        self
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

    pub fn finalize(mut self) -> Result<JointPathPlanner<N>> {
        if let Some(margin) = self.collision_check_margin {
            self.robot_collision_detector.collision_detector.prediction = margin;
        }

        match self.reference_robot {
            Some(robot) => {
                let mut planner = JointPathPlanner::new(
                    robot,
                    self.robot_collision_detector,
                    self.step_length,
                    self.max_try,
                    self.num_smoothing,
                );
                planner.robot_collision_detector.self_collision_pairs = self.self_collision_pairs;
                Ok(planner)
            }
            None => Err(Error::ReferenceRobot("JointPathBuilder".to_owned())),
        }
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

#[derive(Clone, Serialize, Deserialize, Debug, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct JointPathPlannerConfig {
    #[serde(default = "default_step_length")]
    step_length: f64,
    #[serde(default = "default_max_try")]
    max_try: usize,
    #[serde(default = "default_num_smoothing")]
    num_smoothing: usize,
    #[serde(default = "default_margin")]
    margin: f64,
}

fn default_step_length() -> f64 {
    0.1
}

fn default_max_try() -> usize {
    5000
}

fn default_num_smoothing() -> usize {
    100
}

fn default_margin() -> f64 {
    0.001
}

impl Default for JointPathPlannerConfig {
    fn default() -> Self {
        Self {
            step_length: default_step_length(),
            max_try: default_max_try(),
            num_smoothing: default_num_smoothing(),
            margin: default_margin(),
        }
    }
}

pub fn create_joint_path_planner<P: AsRef<Path>>(
    urdf_path: P,
    self_collision_check_pairs: Vec<(String, String)>,
    config: &JointPathPlannerConfig,
    robot: Arc<k::Chain<f64>>,
) -> JointPathPlanner<f64> {
    JointPathPlannerBuilder::from_urdf_file(urdf_path)
        .unwrap()
        .step_length(config.step_length)
        .max_try(config.max_try)
        .num_smoothing(config.num_smoothing)
        .collision_check_margin(config.margin)
        .self_collision_pairs(self_collision_check_pairs)
        .reference_robot(robot)
        .finalize()
        .unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::create_all_collision_pairs;

    #[test]
    fn from_urdf() {
        let urdf_path = Path::new("sample.urdf");
        let _planner = JointPathPlannerBuilder::from_urdf_file(urdf_path)
            .unwrap()
            .collision_check_margin(0.01)
            .finalize();
    }

    #[test]
    fn test_create_joint_path_planner() {
        let urdf_path = Path::new("sample.urdf");
        let robot = k::Chain::from_urdf_file(urdf_path).unwrap();
        let planner = create_joint_path_planner(
            urdf_path,
            create_all_collision_pairs(&robot),
            &JointPathPlannerConfig::default(),
            Arc::new(robot),
        );

        let l_shoulder_yaw_node = planner
            .collision_check_robot()
            .find("l_shoulder_yaw")
            .unwrap();
        let using_joints = k::SerialChain::from_end(l_shoulder_yaw_node);
        let using_joint_names = using_joints
            .iter_joints()
            .map(|j| j.name.to_owned())
            .collect::<Vec<String>>();

        assert!(planner
            .plan_avoid_self_collision(using_joint_names.as_slice(), &[0.0], &[0.0],)
            .is_ok());
        // an error occurs in the start
        assert!(planner
            .plan_avoid_self_collision(using_joint_names.as_slice(), &[1.57], &[0.0],)
            .is_err());
        // an error occurs in the goal
        assert!(planner
            .plan_avoid_self_collision(using_joint_names.as_slice(), &[0.0], &[1.57],)
            .is_err());
    }

    // This test potentially fails because of RRT-based planning,
    // i.e. the success rate is not 100%.
    // Therefore, this test is treated as a `flaky test`.
    #[flaky_test::flaky_test]
    fn test_plan_avoid_self_collision() {
        let urdf_path = Path::new("sample.urdf");
        let robot = k::Chain::from_urdf_file(urdf_path).unwrap();

        // cross the arms only by moving the right arm
        k::SerialChain::from_end(robot.find("r_tool_fixed").unwrap())
            .set_joint_positions_clamped(&[0.9, 0.0, 0.0, 0.0, 0.67, 0.0]);

        let planner = create_joint_path_planner(
            urdf_path,
            create_all_collision_pairs(&robot),
            &JointPathPlannerConfig::default(),
            Arc::new(robot),
        );

        let l_tool_node = planner
            .collision_check_robot()
            .find("l_tool_fixed")
            .unwrap();
        let using_joints = k::SerialChain::from_end(l_tool_node);
        let using_joint_names = using_joints
            .iter_joints()
            .map(|j| j.name.to_owned())
            .collect::<Vec<String>>();

        // an error occurs since the arms collide
        assert!(planner
            .plan_avoid_self_collision(using_joint_names.as_slice(), &[0.0; 6], &[0.0; 6],)
            .is_err());
        // the planner PROBABLY generates a trajectory avoiding self-collisions
        assert!(planner
            .plan_avoid_self_collision(
                using_joint_names.as_slice(),
                &[0.0, -0.3, 0.0, 0.0, 0.0, 0.0],
                &[0.0, 0.3, 0.0, 0.0, 0.0, 0.0],
            )
            .is_ok());
    }
}
