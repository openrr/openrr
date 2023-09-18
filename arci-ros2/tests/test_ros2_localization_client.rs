#![cfg(feature = "ros2")]

use arci::Localization;
use arci_ros2::{r2r, Ros2LocalizationClient};
use assert_approx_eq::assert_approx_eq;
use futures::StreamExt;
use r2r::{
    geometry_msgs::msg::{Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion},
    std_msgs::msg::Header,
    QosProfile,
};

const AMCL_POSE_TOPIC: &str = "/amcl_pose";
const NO_MOTION_UPDATE_SERVICE: &str = "request_nomotion_update";

#[tokio::test(flavor = "multi_thread")]
async fn test_localization_client() {
    let ctx = r2r::Context::create().unwrap();
    let mut node = r2r::Node::create(ctx.clone(), "test_localization_node", "arci_ros2").unwrap();

    let pose_publisher = node
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
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        }
    });

    let client = Ros2LocalizationClient::new(ctx, false, NO_MOTION_UPDATE_SERVICE, AMCL_POSE_TOPIC);

    let current_pose = client.current_pose("").unwrap();

    assert_approx_eq!(current_pose.translation.vector.x, 2.0);
    assert_approx_eq!(current_pose.translation.vector.y, 3.0);
    assert_approx_eq!(current_pose.rotation.angle(), std::f64::consts::FRAC_PI_2);
}

#[tokio::test(flavor = "multi_thread")]
async fn test_localization_client_nomotion_update() {
    let ctx = r2r::Context::create().unwrap();
    let mut node = r2r::Node::create(ctx.clone(), "test_localization_node", "arci_ros2").unwrap();

    let service_server = node
        .create_service::<r2r::std_srvs::srv::Empty::Service>(NO_MOTION_UPDATE_SERVICE)
        .unwrap();

    let client = Ros2LocalizationClient::new(ctx, true, NO_MOTION_UPDATE_SERVICE, AMCL_POSE_TOPIC);

    tokio::spawn(async move {
        loop {
            match service_server.next().await {
                Some(req) => todo!(),
                None => todo!(),
            }
        }
    });

    client.requst_nomotion_update().await;
}
