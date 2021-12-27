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
use std::path::{Path, PathBuf};

use k::nalgebra as na;
use ncollide3d::shape::Compound;
use openrr_planner::FromUrdf;
use structopt::StructOpt;
use urdf_rs::Robot;
use urdf_viz::{kiss3d::window::Window, Action, Key, Modifiers, WindowEvent};

struct CollisionAvoidApp {
    planner: openrr_planner::JointPathPlannerWithIk<
        f64,
        openrr_planner::RandomInitializeIkSolver<f64, openrr_planner::JacobianIkSolver<f64>>,
    >,
    obstacles: Compound<f64>,
    ik_target_pose: na::Isometry3<f64>,
    colliding_link_names: Vec<String>,
    viewer: urdf_viz::Viewer,
    window: Option<Window>,
    arm: k::SerialChain<f64>,
    end_link_name: String,
    ignore_rotation_x: bool,
    ignore_rotation_y: bool,
    ignore_rotation_z: bool,
    self_collision_pairs: Vec<(String, String)>,
    urdf_robot: Robot,
}

impl CollisionAvoidApp {
    fn new(
        robot_path: &Path,
        end_link_name: &str,
        obstacle_path: &Path,
        ignore_rotation_x: bool,
        ignore_rotation_y: bool,
        ignore_rotation_z: bool,
        self_collision_pairs: Vec<(String, String)>,
    ) -> Self {
        let planner = openrr_planner::JointPathPlannerBuilder::from_urdf_file(&robot_path)
            .unwrap()
            .collision_check_margin(0.01f64)
            .finalize();
        let solver = openrr_planner::JacobianIkSolver::new(0.001f64, 0.005, 0.2, 100);
        let solver = openrr_planner::RandomInitializeIkSolver::new(solver, 100);
        let planner = openrr_planner::JointPathPlannerWithIk::new(planner, solver);
        let (mut viewer, mut window) = urdf_viz::Viewer::new("openrr_planner: example reach");
        let urdf_robot =
            urdf_rs::utils::read_urdf_or_xacro(&robot_path).expect("robot file not found");
        viewer.add_robot_with_base_dir(&mut window, &urdf_robot, robot_path.parent());
        viewer.add_axis_cylinders(&mut window, "origin", 1.0);

        let urdf_obstacles =
            urdf_rs::utils::read_urdf_or_xacro(obstacle_path).expect("obstacle file not found");
        let obstacles = Compound::from_urdf_robot(&urdf_obstacles);
        viewer.add_robot(&mut window, &urdf_obstacles);
        println!(
            "robot={}",
            planner.path_planner.robot_collision_detector.robot
        );
        let arm = {
            let end_link = planner
                .path_planner
                .robot_collision_detector
                .robot
                .find(end_link_name)
                .unwrap_or_else(|| panic!("{} not found", end_link_name));
            k::SerialChain::from_end(end_link)
        };
        let ik_target_pose = arm.end_transform();
        let end_link_name = end_link_name.to_owned();
        viewer.add_axis_cylinders(&mut window, "ik_target", 0.3);
        CollisionAvoidApp {
            viewer,
            window: Some(window),
            obstacles,
            ik_target_pose,
            colliding_link_names: Vec::new(),
            planner,
            arm,
            end_link_name,
            ignore_rotation_x,
            ignore_rotation_y,
            ignore_rotation_z,
            self_collision_pairs,
            urdf_robot,
        }
    }

    fn update_robot(&mut self) {
        // this is hack to handle invalid mimic joints
        let ja = self
            .planner
            .path_planner
            .robot_collision_detector
            .robot
            .joint_positions();
        self.planner
            .path_planner
            .robot_collision_detector
            .robot
            .set_joint_positions_clamped(&ja);
        self.viewer
            .update(&self.planner.path_planner.robot_collision_detector.robot);
    }

    fn update_ik_target(&mut self) {
        if let Some(obj) = self.viewer.scene_node_mut("ik_target") {
            obj.set_local_transformation(na::convert(self.ik_target_pose));
        }
    }

    fn reset_colliding_link_colors(&mut self) {
        for link in &self.colliding_link_names {
            self.viewer.reset_temporal_color(link);
        }
    }

    fn run(mut self) {
        let mut is_collide_show = false;

        let c = k::Constraints {
            rotation_x: !self.ignore_rotation_x,
            rotation_y: !self.ignore_rotation_y,
            rotation_z: !self.ignore_rotation_z,
            ..Default::default()
        };

        self.update_robot();
        self.update_ik_target();
        let mut plans: Vec<Vec<f64>> = Vec::new();
        let mut window = self.window.take().unwrap();

        while window.render_with_camera(&mut self.viewer.arc_ball) {
            if !plans.is_empty() {
                self.arm.set_joint_positions(&plans.pop().unwrap()).unwrap();
                self.update_robot();
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
            for event in window.events().iter() {
                if let WindowEvent::Key(code, Action::Press, mods) = event.value {
                    match code {
                        Key::U => {
                            self.ik_target_pose = self.arm.end_transform();
                            self.update_ik_target();
                        }
                        Key::Up => {
                            if mods.contains(Modifiers::Shift) {
                                self.ik_target_pose.rotation *=
                                    na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.2);
                            } else {
                                self.ik_target_pose.translation.vector[2] += 0.05;
                            }
                            self.update_ik_target();
                        }
                        Key::Down => {
                            if mods.contains(Modifiers::Shift) {
                                self.ik_target_pose.rotation *=
                                    na::UnitQuaternion::from_euler_angles(0.0, 0.0, -0.2);
                            } else {
                                self.ik_target_pose.translation.vector[2] -= 0.05;
                            }
                            self.update_ik_target();
                        }
                        Key::Left => {
                            if mods.contains(Modifiers::Shift) {
                                self.ik_target_pose.rotation *=
                                    na::UnitQuaternion::from_euler_angles(0.0, 0.2, -0.0);
                            } else {
                                self.ik_target_pose.translation.vector[1] += 0.05;
                            }
                            self.update_ik_target();
                        }
                        Key::Right => {
                            if mods.contains(Modifiers::Shift) {
                                self.ik_target_pose.rotation *=
                                    na::UnitQuaternion::from_euler_angles(0.0, -0.2, 0.0);
                            } else {
                                self.ik_target_pose.translation.vector[1] -= 0.05;
                            }
                            self.update_ik_target();
                        }
                        Key::B => {
                            if mods.contains(Modifiers::Shift) {
                                self.ik_target_pose.rotation *=
                                    na::UnitQuaternion::from_euler_angles(-0.2, 0.0, 0.0);
                            } else {
                                self.ik_target_pose.translation.vector[0] -= 0.05;
                            }
                            self.update_ik_target();
                        }
                        Key::F => {
                            if mods.contains(Modifiers::Shift) {
                                self.ik_target_pose.rotation *=
                                    na::UnitQuaternion::from_euler_angles(0.2, 0.0, 0.0);
                            } else {
                                self.ik_target_pose.translation.vector[0] += 0.05;
                            }
                            self.update_ik_target();
                        }
                        Key::I => {
                            self.reset_colliding_link_colors();
                            let c = k::Constraints {
                                rotation_x: !self.ignore_rotation_x,
                                rotation_y: !self.ignore_rotation_y,
                                rotation_z: !self.ignore_rotation_z,
                                ..Default::default()
                            };
                            let result = self.planner.solve_ik_with_constraints(
                                &self.arm,
                                &self.ik_target_pose,
                                &c,
                            );
                            if result.is_err() {
                                println!("fail!! {:?}", result);
                            }
                            self.update_robot();
                        }
                        Key::G => {
                            self.reset_colliding_link_colors();
                            match self.planner.plan_with_ik_with_constraints(
                                &self.end_link_name,
                                &self.ik_target_pose,
                                &self.obstacles,
                                &c,
                            ) {
                                Ok(mut plan) => {
                                    plan.reverse();
                                    plans = openrr_planner::interpolate(&plan, 5.0, 0.1)
                                        .unwrap()
                                        .into_iter()
                                        .map(|point| point.position)
                                        .collect();
                                }
                                Err(error) => {
                                    self.update_robot();
                                    println!("failed to reach!! {}", error);
                                }
                            };
                        }
                        Key::R => {
                            self.reset_colliding_link_colors();
                            openrr_planner::set_random_joint_positions(&self.arm).unwrap();
                            self.update_robot();
                        }
                        Key::C => {
                            self.reset_colliding_link_colors();
                            self.colliding_link_names =
                                self.planner.colliding_link_names(&self.obstacles);
                            for name in &self.colliding_link_names {
                                println!("{}", name);
                                self.viewer.set_temporal_color(name, 0.8, 0.8, 0.6);
                            }
                            println!("===========");
                        }
                        Key::S => {
                            self.reset_colliding_link_colors();
                            let pairs: Vec<_> = self
                                .planner
                                .path_planner
                                .robot_collision_detector
                                .collision_detector
                                .detect_self(
                                    &self.planner.path_planner.robot_collision_detector.robot,
                                    &self.self_collision_pairs,
                                )
                                .collect();
                            self.colliding_link_names.clear();
                            for p in pairs {
                                self.colliding_link_names.push(p.0);
                                self.colliding_link_names.push(p.1);
                            }
                            for name in &self.colliding_link_names {
                                println!("{}", name);
                                self.viewer.set_temporal_color(name, 0.8, 0.4, 0.6);
                            }
                            println!("===========");
                        }
                        Key::V => {
                            is_collide_show = !is_collide_show;
                            let ref_robot = &self.urdf_robot;
                            self.viewer.remove_robot(&mut window, ref_robot);
                            self.viewer.add_robot_with_base_dir_and_collision_flag(
                                &mut window,
                                ref_robot,
                                None,
                                is_collide_show,
                            );
                            self.viewer
                                .update(&self.planner.path_planner.robot_collision_detector.robot);
                        }
                        Key::X => {
                            println!("start reachable region calculation");

                            let c = k::Constraints {
                                rotation_x: !self.ignore_rotation_x,
                                rotation_y: !self.ignore_rotation_y,
                                rotation_z: !self.ignore_rotation_z,
                                ..Default::default()
                            };
                            const MAX_X: f64 = 1.5;
                            const MIN_X: f64 = -1.5;
                            const MAX_Y: f64 = 1.5;
                            const MIN_Y: f64 = -1.5;
                            const MAX_Z: f64 = 1.5;
                            const MIN_Z: f64 = -0.5;
                            const UNIT_LENGTH: f64 = 0.2;
                            for v in openrr_planner::get_reachable_region(
                                &self.planner.ik_solver,
                                &self.arm,
                                &self.ik_target_pose,
                                &c,
                                na::Vector3::new(MAX_X, MAX_Y, MAX_Z),
                                na::Vector3::new(MIN_X, MIN_Y, MIN_Z),
                                UNIT_LENGTH,
                            ) {
                                let mut c = window.add_cube(0.05, 0.04, 0.03);
                                c.prepend_to_local_transformation(&na::convert(v));
                                c.set_color(0.0, 1.0, 0.0);
                            }
                            println!("finished reachable region calculation");
                        }
                        _ => {}
                    }
                }
            }
        }
    }
}

#[derive(StructOpt, Debug)]
#[structopt(name = "openrr_planner_example")]
struct Opt {
    #[structopt(short = "x", long = "ignore-rotation-x")]
    ignore_rotation_x: bool,
    #[structopt(short = "y", long = "ignore-rotation-y")]
    ignore_rotation_y: bool,
    #[structopt(short = "z", long = "ignore-rotation-z")]
    ignore_rotation_z: bool,
    #[structopt(
        short = "r",
        long = "robot",
        parse(from_os_str),
        default_value = "sample.urdf"
    )]
    robot_urdf_path: PathBuf,
    #[structopt(
        short = "o",
        long = "obstacle",
        parse(from_os_str),
        default_value = "obstacles.urdf"
    )]
    obstacle_urdf_path: PathBuf,
    #[structopt(short = "e", long = "end-link", default_value = "l_tool_fixed")]
    end_link: String,
    #[structopt(short = "s", long = "self-collision-pair")]
    self_collision_pair: Vec<String>,
}

fn main() -> Result<(), openrr_planner::Error> {
    tracing_subscriber::fmt::init();
    let opt = Opt::from_args();
    let app = CollisionAvoidApp::new(
        &opt.robot_urdf_path,
        &opt.end_link,
        &opt.obstacle_urdf_path,
        opt.ignore_rotation_x,
        opt.ignore_rotation_y,
        opt.ignore_rotation_z,
        openrr_planner::collision::parse_colon_separated_pairs(&opt.self_collision_pair)?,
    );
    app.run();
    Ok(())
}
