mod shared;

use std::time::Duration;

use arci::Localization;
use arci_ros2::{
    msg::{
        geometry_msgs::{Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion},
        std_msgs::{self, Header},
        std_srvs,
    },
    Ros2LocalizationClient,
};
use assert_approx_eq::assert_approx_eq;
use shared::*;

const AMCL_POSE_TOPIC: &str = "/amcl_pose";
const NO_MOTION_UPDATE_SERVICE: &str = "/request_nomotion_update";

#[tokio::test(flavor = "multi_thread")]
async fn test_localization_client() {
    let node = test_node();
    let amcl_pose_topic = node
        .create_topic::<PoseWithCovarianceStamped>(AMCL_POSE_TOPIC)
        .unwrap();
    let pose_publisher = node
        .create_publisher::<PoseWithCovarianceStamped>(&amcl_pose_topic)
        .unwrap();

    tokio::spawn(async move {
        for _ in 0..2 {
            pose_publisher
                .publish(PoseWithCovarianceStamped {
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

    let service_server = node
        .create_server::<std_srvs::Empty>(NO_MOTION_UPDATE_SERVICE)
        .unwrap();

    let client =
        Ros2LocalizationClient::new(node, true, NO_MOTION_UPDATE_SERVICE, AMCL_POSE_TOPIC).unwrap();

    tokio::spawn(async move {
        let (id, _req) = service_server.async_receive_request().await.unwrap();
        service_server
            .async_send_response(id, std_msgs::Empty {})
            .await
            .unwrap();
    });

    client.request_nomotion_update().await;
}
