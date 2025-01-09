#![cfg(feature = "ros2")]

mod shared;

use std::time::Duration;

use arci::MoveBase;
use arci_ros2::{r2r, Ros2CmdVelMoveBase};
use assert_approx_eq::assert_approx_eq;
use r2r::{nav_msgs::msg::Odometry, QosProfile};
use shared::*;

const ODOMETRY_TOPIC: &str = "/odom";

const TWIST_LINEAR_X: f64 = 1.23;
const TWIST_LINEAR_Y: f64 = 2.34;
const TWIST_ANGULAR_Z: f64 = std::f64::consts::FRAC_PI_2;

#[tokio::test(flavor = "multi_thread")]
async fn test_cmd_vel_current_velocity() {
    let node = test_node();
    let odom_publisher = node
        .r2r()
        .create_publisher::<Odometry>(ODOMETRY_TOPIC, QosProfile::default())
        .unwrap();

    tokio::spawn(async move {
        loop {
            let mut odometry = Odometry::default();
            odometry.twist.twist.linear.x = TWIST_LINEAR_X;
            odometry.twist.twist.linear.y = TWIST_LINEAR_Y;
            odometry.twist.twist.angular.z = TWIST_ANGULAR_Z;
            odom_publisher.publish(&odometry).unwrap();
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });

    node.run_spin_thread(Duration::from_millis(100));
    let c = Ros2CmdVelMoveBase::new(node.clone(), "/cmd_vel_test", "/odom");

    let current_velocity = c.current_velocity().unwrap();

    assert_approx_eq!(current_velocity.x, TWIST_LINEAR_X);
    assert_approx_eq!(current_velocity.y, TWIST_LINEAR_Y);
    assert_approx_eq!(current_velocity.theta, TWIST_ANGULAR_Z);
}
