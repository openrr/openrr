mod shared;

use arci::{BaseVelocity, MoveBase};
use arci_ros2::{msg::geometry_msgs::Twist, Ros2CmdVelMoveBase};
use assert_approx_eq::assert_approx_eq;
use shared::*;

#[tokio::test]
async fn test_pub() {
    let node = test_node();
    let c = Ros2CmdVelMoveBase::new(node.clone(), "/cmd_vel_test");

    let topic = node.create_topic::<Twist>("/cmd_vel_test").unwrap();
    let sub = node.create_subscription::<Twist>(&topic).unwrap();

    let mut count = 0;
    let mut vel = BaseVelocity::default();
    while count < 10 {
        vel.x = 0.001 * (count as f64);
        c.send_velocity(&vel).unwrap();
        let v = loop {
            match sub.take().unwrap() {
                Some((v, _)) => break v,
                None => tokio::time::sleep(std::time::Duration::from_millis(10)).await,
            }
        };
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
        let v = loop {
            match sub.take().unwrap() {
                Some((v, _)) => break v,
                None => tokio::time::sleep(std::time::Duration::from_millis(10)).await,
            }
        };
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
