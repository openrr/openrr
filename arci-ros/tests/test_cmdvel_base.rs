#![cfg(target_os = "linux")]
use arci::{BaseVelocity, MoveBase};
mod msg {
    rosrust::rosmsg_include!(geometry_msgs / Twist);
}
use msg::geometry_msgs::Twist;
mod util;
use arci_ros::subscribe_with_channel;
use util::run_roscore_and_rosrust_init_once;

#[tokio::test]
async fn test_cmd_vel() {
    use assert_approx_eq::assert_approx_eq;

    println!("test cmd vel init");
    let _roscore = run_roscore_and_rosrust_init_once("arci_ros_cmd_vel_test");

    let topic_name = String::from("test_twist");
    let (rx, _sub) = subscribe_with_channel::<Twist>(&topic_name, 1);

    let c = arci_ros::RosCmdVelMoveBase::new(&topic_name);
    let mut vel = BaseVelocity::default();
    println!("test cmd_vel is running!");

    for count in 0..100 {
        vel.x = 0.001 * (count as f64);
        c.send_velocity(&vel).unwrap();
        std::thread::sleep(std::time::Duration::from_millis(100));
        println!("{}, {:?}", count, vel);
    }

    let mut rv_count = 0_usize;
    while let Ok(rv) = rx.recv() {
        if rv_count == 99 {
            break;
        } else {
            assert_approx_eq!(rv.linear.x, 0.001 * (rv_count as f64));
            println!("{:?}", rv);
        }
        rv_count += 1;
    }
}
