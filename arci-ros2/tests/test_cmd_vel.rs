#![cfg(feature = "ros2")]

use futures::stream::StreamExt;

#[tokio::test]
async fn test_pub() {
    //use std::sync::mpsc;

    use arci::{BaseVelocity, MoveBase};
    use arci_ros2::{r2r, Ros2CmdVelMoveBase};
    use assert_approx_eq::assert_approx_eq;
    use r2r::geometry_msgs::msg::Twist;

    let ctx = r2r::Context::create().unwrap();
    let c = Ros2CmdVelMoveBase::new(ctx.clone(), "/cmd_vel_test");
    let mut node = r2r::Node::create(ctx, "recv", "arci_ros2_test").unwrap();

    let mut sub = node.subscribe::<Twist>("/cmd_vel_test").unwrap();

    let mut count = 0;
    let mut vel = BaseVelocity::default();
    while count < 10 {
        vel.x = 0.001 * (count as f64);
        c.send_velocity(&vel).unwrap();
        node.spin_once(std::time::Duration::from_millis(10));
        let v = sub.next().await.unwrap();
        assert_approx_eq!(v.linear.x, vel.x);
        assert_approx_eq!(v.linear.y, vel.y);
        assert_approx_eq!(v.linear.z, 0.0);
        assert_approx_eq!(v.angular.x, 0.0);
        assert_approx_eq!(v.angular.y, 0.0);
        assert_approx_eq!(v.angular.z, vel.theta);

        count += 1;
        println!("{}, {:?}", count, vel);
    }
    while count >= 0 {
        vel.x = 0.001 * (count as f64);
        vel.y = 0.002 * (count as f64);
        vel.theta = -0.003 * (count as f64);
        c.send_velocity(&vel).unwrap();
        node.spin_once(std::time::Duration::from_millis(10));
        let v = sub.next().await.unwrap();
        assert_approx_eq!(v.linear.x, vel.x);
        assert_approx_eq!(v.linear.y, vel.y);
        assert_approx_eq!(v.linear.z, 0.0);
        assert_approx_eq!(v.angular.x, 0.0);
        assert_approx_eq!(v.angular.y, 0.0);
        assert_approx_eq!(v.angular.z, vel.theta);

        count -= 1;
        println!("{}, {:?}", count, vel);
    }
}
