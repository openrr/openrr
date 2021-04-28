use std::sync::mpsc;

use arci::{BaseVelocity, MoveBase};
use arci_ros2::{r2r, Ros2CmdVelMoveBase};
use assert_approx_eq::assert_approx_eq;
use r2r::geometry_msgs::msg::Twist;

#[tokio::test]
async fn test_pub() {
    let ctx = r2r::Context::create().unwrap();
    let mut node = r2r::Node::create(ctx, "example_cmd_vel_node", "").unwrap();
    let c = Ros2CmdVelMoveBase::new(&mut node, "/cmd_vel_test");
    let (tx, rx) = mpsc::channel::<Twist>();

    let _sub = node.subscribe(
        "/cmd_vel_test",
        Box::new(move |v: Twist| {
            tx.send(v).unwrap();
        }),
    );

    let mut count = 0;
    let mut vel = BaseVelocity::default();
    while count < 10 {
        vel.x = 0.001 * (count as f64);
        c.send_velocity(&vel).unwrap();
        node.spin_once(std::time::Duration::from_millis(10));
        let v = rx.recv().unwrap();
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
        let v = rx.recv().unwrap();
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
