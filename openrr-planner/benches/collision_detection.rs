use std::{f64::consts::PI, path::Path};

use criterion::{criterion_group, criterion_main, Criterion};
use k::joint::Range;
use na::RealField;
use nalgebra as na;
use openrr_planner::collision::{
    create_all_collision_pairs, create_robot_collision_detector, RobotCollisionDetectorConfig,
};

fn generate_random_joint_angles_from_limits<T>(limits: &[Option<k::joint::Range<T>>]) -> Vec<T>
where
    T: RealField,
{
    limits
        .iter()
        .map(|range| match range {
            Some(range) => {
                (range.max.clone() - range.min.clone()) * na::convert(rand::random())
                    + range.min.clone()
            }
            None => na::convert::<f64, T>(rand::random::<f64>() - 0.5) * na::convert(2.0 * PI),
        })
        .collect()
}

fn bench_check_environmental_collisions(c: &mut Criterion) {
    let urdf_path = Path::new("sample.urdf");
    let urdf_robot = urdf_rs::read_file(urdf_path).unwrap();
    let robot = k::Chain::<f64>::from(&urdf_robot);
    let collision_pairs = create_all_collision_pairs(&robot);

    let robot_collision_detector = create_robot_collision_detector(
        urdf_path,
        RobotCollisionDetectorConfig::default(),
        collision_pairs,
    );
    let limits = robot_collision_detector
        .robot
        .iter_joints()
        .map(|j| j.limits)
        .collect::<Vec<Option<Range<f64>>>>();

    let obstacle = openrr_planner::FromUrdf::from_urdf_file("obstacles.urdf").unwrap();

    c.bench_function("bench_check_environmental_collisions", |b| {
        b.iter(|| {
            // Set random joint positions to the robot
            let angles = generate_random_joint_angles_from_limits(&limits);
            robot_collision_detector
                .robot
                .set_joint_positions(&angles)
                .unwrap();
            robot_collision_detector.robot.update_transforms();

            // Check collisions
            robot_collision_detector.env_collision_link_names(&obstacle);
        });
    });
}

fn bench_check_self_collisions(c: &mut Criterion) {
    let urdf_path = Path::new("sample.urdf");
    let urdf_robot = urdf_rs::read_file(urdf_path).unwrap();
    let robot = k::Chain::<f64>::from(&urdf_robot);
    let collision_pairs = create_all_collision_pairs(&robot);

    let robot_collision_detector = create_robot_collision_detector(
        urdf_path,
        RobotCollisionDetectorConfig::default(),
        collision_pairs,
    );
    let limits = robot_collision_detector
        .robot
        .iter_joints()
        .map(|j| j.limits)
        .collect::<Vec<Option<Range<f64>>>>();

    c.bench_function("bench_check_self_collisions", |b| {
        b.iter(|| {
            // Set random joint positions to the robot
            let angles = generate_random_joint_angles_from_limits(&limits);
            robot_collision_detector
                .robot
                .set_joint_positions(&angles)
                .unwrap();
            robot_collision_detector.robot.update_transforms();

            // Check collisions
            robot_collision_detector.self_collision_link_pairs();
        });
    });
}

criterion_group!(
    name = benches;
    config = Criterion::default().sample_size(100);
    targets = bench_check_environmental_collisions, bench_check_self_collisions);
criterion_main!(benches);
