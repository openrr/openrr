#![cfg(target_os = "linux")]

use arci::{JointTrajectoryClient, TrajectoryPoint};
mod msg {
    rosrust::rosmsg_include!(sensor_msgs / JointState, trajectory_msgs / JointTrajectory);
}
use arci_ros::subscribe_with_channel;
use msg::{sensor_msgs, trajectory_msgs};
mod util;
use assert_approx_eq::assert_approx_eq;
use util::run_roscore_and_rosrust_init_once;

const JOINT_STATE_TOPIC_NAME: &str = "joint_state";
const TRAJECTORY_TOPIC_NAME: &str = "trajectory";

#[test]
fn test_robot_client() {
    println!("arci ros robot client test");

    // Setup for ROS
    let _roscore = run_roscore_and_rosrust_init_once("arci_ros_robot_client_test");

    let joint_names = vec![
        String::from("joint0"),
        String::from("joint1"),
        String::from("joint2"),
    ];

    let joint_state_publisher =
        rosrust::publish::<sensor_msgs::JointState>(JOINT_STATE_TOPIC_NAME, 1).unwrap();

    let (_rx, joint_trajectory_subscriber) =
        subscribe_with_channel::<trajectory_msgs::JointTrajectory>(TRAJECTORY_TOPIC_NAME, 1);

    std::thread::spawn(move || {
        // Wait for ros_robot_client to be launched.
        while rosrust::is_ok() && joint_state_publisher.subscriber_count() == 0 {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        joint_state_publisher
            .send(sensor_msgs::JointState {
                header: Default::default(),
                name: vec![
                    String::from("joint0"),
                    String::from("joint1"),
                    String::from("joint2"),
                ],
                position: vec![0f64, 1f64, 2f64],
                velocity: vec![3f64, 4f64, 5f64],
                effort: vec![6f64, 7f64, 8f64],
            })
            .unwrap();

        while rosrust::is_ok() && joint_trajectory_subscriber.publisher_count() == 0 {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    });

    let client =
        arci_ros::RosRobotClient::new(joint_names, JOINT_STATE_TOPIC_NAME, TRAJECTORY_TOPIC_NAME);

    assert_eq!(client.joint_names()[0], "joint0");
    assert_approx_eq!(client.current_joint_positions().unwrap()[1], 1f64);

    assert!(
        client
            .send_joint_positions(vec![0.5, 1.5, 2.5], std::time::Duration::from_millis(100),)
            .is_ok()
    );

    assert!(
        client
            .send_joint_trajectory(vec![TrajectoryPoint {
                positions: vec![1.0, 2.0, 3.0],
                velocities: Some(vec![3.5, 4.5, 5.5]),
                time_from_start: std::time::Duration::from_millis(100),
            }])
            .is_ok()
    );
}
