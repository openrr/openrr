#![cfg(feature = "ros2")]

mod shared;

use std::time::Duration;

use arci::Localization;
use arci_ros2::{r2r, Ros2LocalizationClient};
use assert_approx_eq::assert_approx_eq;
use futures::StreamExt;
use r2r::{
    geometry_msgs::msg::{Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion},
    std_msgs::msg::Header,
    QosProfile,
};
use shared::*;

const AMCL_POSE_TOPIC: &str = "/amcl_pose";
const NO_MOTION_UPDATE_SERVICE: &str = "request_nomotion_update";

#[tokio::test(flavor = "multi_thread")]
async fn test_localization_client() {
    let node = test_node();

    let pose_publisher = node
        .r2r()
        .create_publisher::<PoseWithCovarianceStamped>(AMCL_POSE_TOPIC, QosProfile::default())
        .unwrap();

    tokio::spawn(async move {
        loop {
            pose_publisher
                .publish(&PoseWithCovarianceStamped {
                    header: Header::default(),
                    pose: PoseWithCovariance {
                        pose: Pose {
                            position: Point {
                                x: 2.0,
                                y: 3.0,
                                z: 4.0,
                            },
                            orientation: Quaternion {
                                x: 0.0,
                                y: 0.0,
                                z: 0.5 * std::f64::consts::SQRT_2,
                                w: 0.5 * std::f64::consts::SQRT_2,
                            },
                        },
                        covariance: vec![1f64; 36],
                    },
                })
                .unwrap();
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });

    node.run_spin_thread(Duration::from_millis(100));
    let client =
        Ros2LocalizationClient::new(node, false, NO_MOTION_UPDATE_SERVICE, AMCL_POSE_TOPIC)
            .unwrap();

    let current_pose = client.current_pose("").unwrap();

    assert_approx_eq!(current_pose.translation.vector.x, 2.0);
    assert_approx_eq!(current_pose.translation.vector.y, 3.0);
    assert_approx_eq!(current_pose.rotation.angle(), std::f64::consts::FRAC_PI_2);
}

#[tokio::test(flavor = "multi_thread")]
async fn test_localization_client_nomotion_update() {
    let node = test_node();

    let mut service_server = node
        .r2r()
        .create_service::<r2r::std_srvs::srv::Empty::Service>(
            NO_MOTION_UPDATE_SERVICE,
            QosProfile::default(),
        )
        .unwrap();

    node.run_spin_thread(Duration::from_millis(100));
    let client =
        Ros2LocalizationClient::new(node, true, NO_MOTION_UPDATE_SERVICE, AMCL_POSE_TOPIC).unwrap();

    tokio::spawn(async move {
        let req = service_server.next().await.unwrap();
        req.respond(r2r::std_srvs::srv::Empty::Response {}).unwrap();
    });

    client.request_nomotion_update().await;
}
