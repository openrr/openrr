#![cfg(feature = "ros2")]

mod shared;

use arci::{BaseVelocity, MoveBase};
use arci_ros2::{r2r, Ros2CmdVelMoveBase};
use assert_approx_eq::assert_approx_eq;
use futures::stream::StreamExt;
use r2r::geometry_msgs::msg::Twist;
use shared::*;

#[tokio::test]
async fn test_pub() {
    let node = test_node();
    let c = Ros2CmdVelMoveBase::new(node.clone(), "/cmd_vel_test", "/odom");

    let mut sub = node
        .r2r()
        .subscribe::<Twist>("/cmd_vel_test", r2r::QosProfile::default())
        .unwrap();

    let mut count = 0;
    let mut vel = BaseVelocity::default();
    while count < 10 {
        vel.x = 0.001 * (count as f64);
        c.send_velocity(&vel).unwrap();
        node.spin_once(std::time::Duration::from_millis(10)).await;
        let v = sub.next().await.unwrap();
        assert_approx_eq!(v.linear.x, vel.x);
        assert_approx_eq!(v.linear.y, vel.y);
        assert_approx_eq!(v.linear.z, 0.0);
        assert_approx_eq!(v.angular.x, 0.0);
        assert_approx_eq!(v.angular.y, 0.0);
        assert_approx_eq!(v.angular.z, vel.theta);

        count += 1;
        println!("{count}, {vel:?}");
    }
    while count >= 0 {
        vel.x = 0.001 * (count as f64);
        vel.y = 0.002 * (count as f64);
        vel.theta = -0.003 * (count as f64);
        c.send_velocity(&vel).unwrap();
        node.spin_once(std::time::Duration::from_millis(10)).await;
        let v = sub.next().await.unwrap();
        assert_approx_eq!(v.linear.x, vel.x);
        assert_approx_eq!(v.linear.y, vel.y);
        assert_approx_eq!(v.linear.z, 0.0);
        assert_approx_eq!(v.angular.x, 0.0);
        assert_approx_eq!(v.angular.y, 0.0);
        assert_approx_eq!(v.angular.z, vel.theta);

        count -= 1;
        println!("{count}, {vel:?}");
    }
}
