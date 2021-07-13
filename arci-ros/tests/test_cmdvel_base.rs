#![cfg(target_os = "linux")]
use arci::{BaseVelocity, MoveBase};

mod msg {
    rosrust::rosmsg_include!(geometry_msgs / Twist);
}
use msg::geometry_msgs::Twist;
mod util;

#[tokio::test]
async fn test_cmd_vel() {
    use std::sync::mpsc;

    use assert_approx_eq::assert_approx_eq;

    println!("test cmd vel init");
    let _roscore = util::run_roscore_for(util::Language::None, util::Feature::Publisher);

    let topic_name = String::from("test_twist");
    arci_ros::init("arci_ros_cmd_vel_test");
    let (tx, rx) = mpsc::channel::<Twist>();

    let _sub = rosrust::subscribe(&topic_name, 1, move |v: Twist| {
        println!("{:?}", v);
        tx.send(v).unwrap();
    })
    .unwrap();

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
