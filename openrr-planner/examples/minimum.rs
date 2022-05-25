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

use k::nalgebra as na;
use ncollide3d::shape::Compound;
use openrr_planner::FromUrdf;

fn main() {
    // Create path planner with loading urdf file and set end link name
    let planner = openrr_planner::JointPathPlannerBuilder::from_urdf_file("sample.urdf")
        .expect("failed to create planner from urdf file")
        .collision_check_margin(0.01)
        .finalize();
    // Create inverse kinematics solver
    let solver = openrr_planner::JacobianIkSolver::default();
    let solver = openrr_planner::RandomInitializeIkSolver::new(solver, 100);
    // Create path planner with IK solver
    let mut planner = openrr_planner::JointPathPlannerWithIk::new(planner, solver);
    let target_name = "l_wrist_pitch";
    // Create obstacles
    let obstacles = Compound::from_urdf_file("obstacles.urdf").expect("obstacle file not found");

    // Set IK target transformation
    let mut ik_target_pose = na::Isometry3::from_parts(
        na::Translation3::new(0.4, 0.4, 0.0),
        na::UnitQuaternion::from_euler_angles(0.0, -1.57, 0.0),
    );

    // Plan the path, path is the vector of joint angles for root to target_name
    let plan1 = planner
        .plan_with_ik(target_name, &ik_target_pose, &obstacles)
        .unwrap();
    println!("plan1 = {plan1:?}");

    // Plan the path from previous result
    ik_target_pose.translation.vector[0] += 0.1;
    let plan2 = planner
        .plan_with_ik(target_name, &ik_target_pose, &obstacles)
        .unwrap();
    println!("plan2 = {plan2:?}");
}
