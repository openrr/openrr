#[cfg(target_os = "linux")]
mod msg {
    rosrust::rosmsg_include!(
        geometry_msgs / PoseWithCovarianceStamped,
        geometry_msgs / Point,
        geometry_msgs / Pose,
        geometry_msgs / Quaternion,
        std_msgs / Header
    );
}
use arci::Localization;

mod util;
use assert_approx_eq::assert_approx_eq;
use util::run_roscore_and_rosrust_init_once;

use crate::msg::{geometry_msgs, std_msgs};

const AMCL_POSE_TOPIC: &str = "/amcl_pose";
const NO_MOTION_UPDATE_SERVICE: &str = "request_nomotion_update";

#[test]
fn test_localization_client() {
    println!("arci ros localization client test");

    // Setup for ROS
    let _roscore = run_roscore_and_rosrust_init_once("arci_ros_localization_client_test");

    let pose_publisher =
        rosrust::publish::<geometry_msgs::PoseWithCovarianceStamped>(AMCL_POSE_TOPIC, 1).unwrap();

    std::thread::spawn(move || {
        while rosrust::is_ok() && pose_publisher.subscriber_count() == 0 {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        pose_publisher
            .send(geometry_msgs::PoseWithCovarianceStamped {
                header: std_msgs::Header::default(),
                pose: geometry_msgs::PoseWithCovariance {
                    pose: geometry_msgs::Pose {
                        position: geometry_msgs::Point {
                            x: 2.0,
                            y: 3.0,
                            z: 4.0,
                        },
                        orientation: geometry_msgs::Quaternion {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                            w: 1.0,
                        },
                    },
                    covariance: [1f64; 36],
                },
            })
            .unwrap();
    });

    let client = arci_ros::RosLocalizationClient::new(
        false,
        NO_MOTION_UPDATE_SERVICE.to_string(),
        AMCL_POSE_TOPIC.to_string(),
    );

    let pose = client.current_pose("frame_id").unwrap();

    assert_approx_eq!(pose.rotation.re, 1.0);
    assert_approx_eq!(pose.rotation.im, 0.0);
    assert_approx_eq!(pose.translation.x, 2.0);
    assert_approx_eq!(pose.translation.y, 3.0);
}
