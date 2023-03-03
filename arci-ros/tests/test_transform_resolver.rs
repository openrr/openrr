#![cfg(target_os = "linux")]

mod msg {
    rosrust::rosmsg_include!(geometry_msgs / TransformStamped, tf2_msgs / TFMessage);
}

use arci::TransformResolver;
use msg::{geometry_msgs, tf2_msgs};
mod util;
use arci_ros::convert_ros_time_to_system_time;
use assert_approx_eq::assert_approx_eq;
use util::run_roscore_and_rosrust_init_once;

use crate::msg::geometry_msgs::{Quaternion, Vector3};

const RETRY_RATE: f64 = 100.0;
const MAX_RETRY: usize = 100;

const FRAME_ID_1: &str = "map";
const FRAME_ID_2: &str = "odom";
const FRAME_ID_3: &str = "base_link";
const TF_TRA_X: f64 = 1.2;
const TF_TRA_Y: f64 = 2.3;
const TF_TRA_Z: f64 = 3.4;

// Roll: 0, Pitch: 0, Yaw: 1.5707963(=PI/2)
const TF_ROT_W: f64 = std::f64::consts::FRAC_1_SQRT_2;
const TF_ROT_X: f64 = 0.;
const TF_ROT_Y: f64 = 0.;
const TF_ROT_Z: f64 = std::f64::consts::FRAC_1_SQRT_2;

// Transform from "base_link" to "map"
const EXPECTED_TRA_X: f64 = -1.1;
const EXPECTED_TRA_Y: f64 = 3.5;
const EXPECTED_TRA_Z: f64 = -6.8;
const EXPECTED_ROT_W: f64 = 0.;
const EXPECTED_ROT_X: f64 = 0.;
const EXPECTED_ROT_Y: f64 = 0.;
const EXPECTED_ROT_Z: f64 = -1.;

#[test]
fn test_transform_resolver() {
    println!("arci ros tf resolver test");

    // Setup for ROS
    let _roscore = run_roscore_and_rosrust_init_once("arci_ros_tf_resolver_test");

    let tf_publisher = rosrust::publish::<tf2_msgs::TFMessage>("/tf", 1000).unwrap();

    let ros_transform_resolver = arci_ros::RosTransformResolver::new(
        std::time::Duration::from_millis(100),
        RETRY_RATE,
        MAX_RETRY,
    );

    let dummy_ros_time_first = rosrust::now();
    std::thread::sleep(std::time::Duration::from_millis(10));
    let dummy_ros_time_middle = rosrust::now();
    std::thread::sleep(std::time::Duration::from_millis(10));
    let dummy_ros_time_last = rosrust::now();

    let dummy_time_middle = convert_ros_time_to_system_time(&dummy_ros_time_middle);

    let mut tf_stamped_common = geometry_msgs::TransformStamped::default();
    tf_stamped_common.header.stamp = dummy_ros_time_first;
    tf_stamped_common.transform.translation = Vector3 {
        x: TF_TRA_X,
        y: TF_TRA_Y,
        z: TF_TRA_Z,
    };
    tf_stamped_common.transform.rotation = Quaternion {
        x: TF_ROT_X,
        y: TF_ROT_Y,
        z: TF_ROT_Z,
        w: TF_ROT_W,
    };

    let mut tf_odom_stamped_first = tf_stamped_common.clone();
    tf_odom_stamped_first.header.frame_id = FRAME_ID_1.to_string();
    tf_odom_stamped_first.child_frame_id = FRAME_ID_2.to_string();

    let mut tf_base_stamped_first = tf_stamped_common;
    tf_base_stamped_first.header.frame_id = FRAME_ID_2.to_string();
    tf_base_stamped_first.child_frame_id = FRAME_ID_3.to_string();

    let mut tf_odom_stamped_last = tf_odom_stamped_first.clone();
    tf_odom_stamped_last.header.stamp = dummy_ros_time_last;

    let mut tf_base_stamped_last = tf_base_stamped_first.clone();
    tf_base_stamped_last.header.stamp = dummy_ros_time_last;

    let tf = vec![
        tf_odom_stamped_first,
        tf_base_stamped_first,
        tf_odom_stamped_last,
        tf_base_stamped_last,
    ];
    let tf_message = tf2_msgs::TFMessage { transforms: tf };

    std::thread::spawn(move || {
        while rosrust::is_ok() && tf_publisher.subscriber_count() == 0 {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        std::thread::sleep(std::time::Duration::from_millis(100));

        tf_publisher.send(tf_message).unwrap();
    });

    let tf_received = ros_transform_resolver
        .resolve_transformation(FRAME_ID_3, FRAME_ID_1, dummy_time_middle)
        .unwrap();

    assert_approx_eq!(tf_received.translation.x, EXPECTED_TRA_X);
    assert_approx_eq!(tf_received.translation.y, EXPECTED_TRA_Y);
    assert_approx_eq!(tf_received.translation.z, EXPECTED_TRA_Z);
    assert_approx_eq!(tf_received.rotation.w, EXPECTED_ROT_W);
    assert_approx_eq!(tf_received.rotation.i, EXPECTED_ROT_X);
    assert_approx_eq!(tf_received.rotation.j, EXPECTED_ROT_Y);
    assert_approx_eq!(tf_received.rotation.k, EXPECTED_ROT_Z);
}
