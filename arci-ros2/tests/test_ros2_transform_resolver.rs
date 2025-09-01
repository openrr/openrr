#![cfg(feature = "ros2")]

use std::time::Duration;

use arci::TransformResolver;
use arci_ros2::{
    r2r::{
        QosProfile, builtin_interfaces::msg::Time, geometry_msgs::msg::TransformStamped,
        tf2_msgs::msg::TFMessage,
    },
    utils::convert_ros2_time_to_system_time,
};
use assert_approx_eq::assert_approx_eq;
use r2r::geometry_msgs::msg::{Quaternion, Vector3};

const RETRY_RATE: f64 = 50.0;
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

#[tokio::test(flavor = "multi_thread")]
async fn test_transform_resolver() {
    let tf_node = arci_ros2::Node::new("test_tf", "arci_ros2").unwrap();
    let tf_publisher = tf_node
        .r2r()
        .create_publisher::<TFMessage>("/tf", QosProfile::default())
        .unwrap();

    tf_node.run_spin_thread(Duration::from_millis(100));

    let ros2_transform_resolver = arci_ros2::Ros2TransformResolver::new(
        tf_node.clone(),
        Duration::from_millis(100),
        RETRY_RATE,
        MAX_RETRY,
    );

    let mut ros2_clock = arci_ros2::r2r::Clock::create(r2r::ClockType::RosTime).unwrap();

    let dummy_ros2_time_first = ros2_time_from_duration(ros2_clock.get_now().unwrap());
    std::thread::sleep(Duration::from_millis(10));
    let dummy_ros2_time_middle = ros2_time_from_duration(ros2_clock.get_now().unwrap());
    std::thread::sleep(Duration::from_millis(10));
    let dummy_ros2_time_last = ros2_time_from_duration(ros2_clock.get_now().unwrap());

    let dummy_time_middle = convert_ros2_time_to_system_time(&dummy_ros2_time_middle);

    let mut tf_stamped_common = TransformStamped::default();
    tf_stamped_common.header.stamp = dummy_ros2_time_first;
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
    tf_odom_stamped_last.header.stamp = dummy_ros2_time_last.clone();

    let mut tf_base_stamped_last = tf_base_stamped_first.clone();
    tf_base_stamped_last.header.stamp = dummy_ros2_time_last;

    let tf = vec![
        tf_odom_stamped_first,
        tf_base_stamped_first,
        tf_odom_stamped_last,
        tf_base_stamped_last,
    ];
    let tf_message = TFMessage { transforms: tf };

    tokio::spawn(async move {
        loop {
            tf_publisher.publish(&tf_message).unwrap();
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
    });
    tokio::time::sleep(Duration::from_secs(1)).await;

    let tf_received = ros2_transform_resolver
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

fn ros2_time_from_duration(duration: Duration) -> Time {
    Time {
        sec: duration.as_secs() as i32,
        nanosec: duration.subsec_nanos(),
    }
}
