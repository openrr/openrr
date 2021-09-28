#![cfg(target_os = "linux")]
use std::{format, thread};

mod msg {
    rosrust::rosmsg_include!(geometry_msgs / Twist);
}
use arci_ros::msg::{
    control_msgs::JointTrajectoryControllerState,
    trajectory_msgs::{JointTrajectory, JointTrajectoryPoint},
};
mod util;
use arci_ros::subscribe_with_channel;
use util::run_roscore_and_rosrust_init_once;

/// # Test for generation of control client
///
/// ``Control Client`` needs connecting ``topic_name/state`` , ``topic_name/command``.
/// This test watch the connecting.
/// This is correct pattern and ``Control Client`` can be generated.
///
#[tokio::test]
async fn test_ctrl_client_gen() {
    println!("arci ros ctrl client test");

    // setup for ROS
    let _roscore = run_roscore_and_rosrust_init_once("arci_ros_ctrl_client_test");

    // core topic name
    const CONTROLLER_NAME: &str = "test_ctrl_client";

    // initialize other actor
    let _handle = thread::spawn(move || {
        // connecting topic (``test_ctrl_client/state<JointTrajectoryControllerState>``)
        let (_rx, _sub) = subscribe_with_channel::<JointTrajectoryControllerState>(
            &format!("{}/state", CONTROLLER_NAME),
            1,
        );

        let pos = vec![0.0_f64, 0.0];
        let duration = std::time::Duration::from_millis(1000);
        let actual = JointTrajectoryPoint {
            positions: pos.clone(),
            velocities: vec![0.0; 2],
            time_from_start: duration.into(),
            ..Default::default()
        };
        let desired = JointTrajectoryPoint {
            positions: pos,
            velocities: vec![0.0; 2],
            time_from_start: duration.into(),
            ..Default::default()
        };
        let publisher = rosrust::publish::<JointTrajectoryControllerState>(
            &format!("{}/state", CONTROLLER_NAME),
            1,
        )
        .unwrap();

        // waiting for subscriber of RosControlClient
        while rosrust::is_ok() && publisher.subscriber_count() == 0 {
            thread::sleep(std::time::Duration::from_secs(1));
        }

        publisher
            .send(JointTrajectoryControllerState {
                joint_names: vec!["a".to_owned()],
                actual,
                desired,
                error: JointTrajectoryPoint::default(),
                ..Default::default()
            })
            .unwrap();
        if let Ok(rv) = _rx.recv() {
            println!("mpsc {:?}", rv);
        }
        drop(_sub);

        // connecting topic (``test_ctrl_client/command<JointTrajectory>``)
        let (_rx, sub) =
            subscribe_with_channel::<JointTrajectory>(&format!("{}/command", CONTROLLER_NAME), 1);

        // waiting for publisher of RosControlClient
        while rosrust::is_ok() && sub.publisher_count() == 0 {
            thread::sleep(std::time::Duration::from_secs(1));
        }
    });

    // # construct ``arci_ros::RosControlClient``
    //
    // inner action : waiting for connecting topic
    // - JointTrajectoryControllerState(join controller topic)
    // - JointTrajectory(sending command topic)
    let _ctrl_client = arci_ros::RosControlClient::new(vec!["a".to_owned()], CONTROLLER_NAME, true);
}

/// # Test for generation of control client
///
/// ``Control Client`` needs connecting ``topic_name/state`` , ``topic_name/command``.
/// This test watch the connecting and if it can not find ``topic name``, generator should ``panic!``.
/// This test is expected ``panic!``.
///
#[tokio::test]
#[should_panic]
async fn test_ctrl_client_gen_panic() {
    println!("arci ros ctrl client test panic");

    // setup for ROS
    let _roscore = run_roscore_and_rosrust_init_once("test_ctrl_client_gen_panic");

    // core topic name
    const CONTROLLER_NAME: &str = "test_ctrl_client_panic";

    // initialize other actor
    let _handle = thread::spawn(move || {
        // connecting topic (``test_ctrl_client/state<JointTrajectoryControllerState>``)
        let (_rx, _sub) = subscribe_with_channel::<JointTrajectoryControllerState>(
            &format!("{}/state", CONTROLLER_NAME),
            1,
        );

        let pos = vec![0.0_f64, 0.0];
        let duration = std::time::Duration::from_millis(1000);
        let actual = JointTrajectoryPoint {
            positions: pos.clone(),
            velocities: vec![0.0; 2],
            time_from_start: duration.into(),
            ..Default::default()
        };
        let desired = JointTrajectoryPoint {
            positions: pos,
            velocities: vec![0.0; 2],
            time_from_start: duration.into(),
            ..Default::default()
        };
        let publisher = rosrust::publish::<JointTrajectoryControllerState>(
            &format!("{}/state", CONTROLLER_NAME),
            1,
        )
        .unwrap();

        // waiting for subscriber of RosControlClient
        while rosrust::is_ok() && publisher.subscriber_count() == 0 {
            thread::sleep(std::time::Duration::from_secs(1));
        }

        // This controller should have ``ctrl_client`` 's joint name
        publisher
            .send(JointTrajectoryControllerState {
                joint_names: vec!["b".to_owned()],
                actual,
                desired,
                error: JointTrajectoryPoint::default(),
                ..Default::default()
            })
            .unwrap();
        if let Ok(rv) = _rx.recv() {
            println!("mpsc {:?}", rv);
        }
        drop(_sub);

        // connecting topic (``test_ctrl_client/command<JointTrajectory>``)
        let (_rx, sub) =
            subscribe_with_channel::<JointTrajectory>(&format!("{}/command", CONTROLLER_NAME), 1);

        // waiting for publisher of RosControlClient
        while rosrust::is_ok() && sub.publisher_count() == 0 {
            thread::sleep(std::time::Duration::from_secs(1));
        }
    });

    // # construct ``arci_ros::RosControlClient``
    //
    // inner action : waiting for connecting topic
    // - JointTrajectoryControllerState(join controller topic)
    // - JointTrajectory(sending command topic)
    let _ctrl_client = arci_ros::RosControlClient::new(vec!["a".to_owned()], CONTROLLER_NAME, true);
}
