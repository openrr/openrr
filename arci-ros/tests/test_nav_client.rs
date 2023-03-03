#![cfg(target_os = "linux")]

mod msg {
    rosrust::rosmsg_include!(actionlib_msgs / GoalID, move_base_msgs / MoveBaseActionGoal);
}
mod util;
use arci::{Isometry2, Navigation, Vector2};
use arci_ros::subscribe_with_channel;
use assert_approx_eq::assert_approx_eq;
use util::run_roscore_and_rosrust_init_once;

use crate::msg::{actionlib_msgs, move_base_msgs};

const NO_MOTION_UPDATE_SERVICE: &str = "request_nomotion_update";
const MOVE_BASE_ACTION: &str = "/move_base";
const CLEAR_COSTMAP_SERVICE: &str = "/move_base/clear_costmaps";

const NAV_GOAL_X: f64 = 1.0;
const NAV_GOAL_Y: f64 = 2.0;
const NAV_GOAL_THETA: f64 = std::f64::consts::FRAC_PI_2;
const NAV_GOAL_QUATERNION_W: f64 = std::f64::consts::FRAC_1_SQRT_2;

#[test]
fn test_nav_client() {
    println!("arci ros nav client test");

    // Setup for ROS
    let _roscore = run_roscore_and_rosrust_init_once("arci_ros_nav_client_test");

    let (goal_rx, _goal_subscriber) = subscribe_with_channel::<move_base_msgs::MoveBaseActionGoal>(
        &format!("{}/goal", MOVE_BASE_ACTION),
        1,
    );

    let (_cancel_rx, _cancel_subscriber) = subscribe_with_channel::<actionlib_msgs::GoalID>(
        &format!("{}/cancel", MOVE_BASE_ACTION),
        1,
    );

    let client = arci_ros::RosNavClient::new(
        false,
        MOVE_BASE_ACTION.to_string(),
        NO_MOTION_UPDATE_SERVICE.to_string(),
        CLEAR_COSTMAP_SERVICE.to_string(),
    );

    std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(1000));

        let _wait_future = client
            .send_goal_pose(
                Isometry2::new(Vector2::new(NAV_GOAL_X, NAV_GOAL_Y), NAV_GOAL_THETA),
                Default::default(),
                std::time::Duration::from_millis(1000),
            )
            .unwrap();
    });

    if let Ok(rv) = goal_rx.recv() {
        assert_approx_eq!(rv.goal.target_pose.pose.position.x, NAV_GOAL_X);
        assert_approx_eq!(rv.goal.target_pose.pose.position.y, NAV_GOAL_Y);
        assert_approx_eq!(
            rv.goal.target_pose.pose.orientation.w,
            NAV_GOAL_QUATERNION_W
        );
        println!("[Subscriber] Goal received.");
    }
}
