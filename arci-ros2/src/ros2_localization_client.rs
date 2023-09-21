use arci::{nalgebra as na, *};
use r2r::{geometry_msgs::msg::PoseWithCovarianceStamped, QosProfile};
use serde::{Deserialize, Serialize};
use tracing::info;

use crate::{utils, Node};

/// `arci::Localization` implementation for ROS2.
pub struct Ros2LocalizationClient {
    node: Node,
    nomotion_update_client: Option<r2r::Client<r2r::std_srvs::srv::Empty::Service>>,
    amcl_pose_topic_name: String,
}

impl Ros2LocalizationClient {
    /// Creates a new `Ros2LaserScan2D`.
    pub fn new(
        node: Node,
        request_final_nomotion_update_hack: bool,
        nomotion_update_service_name: &str,
        amcl_pose_topic_name: &str,
    ) -> Self {
        let nomotion_update_client = if request_final_nomotion_update_hack {
            Some(
                node.r2r()
                    .create_client(nomotion_update_service_name)
                    .unwrap(),
            )
        } else {
            None
        };

        Self {
            node,
            nomotion_update_client,
            amcl_pose_topic_name: amcl_pose_topic_name.to_owned(),
        }
    }

    /// Request final nomotion update hack
    pub async fn request_nomotion_update(&self) {
        match self.nomotion_update_client.as_ref() {
            Some(client) => {
                let is_available = self.node.r2r().is_available(client).unwrap();
                is_available.await.unwrap();

                client
                    .request(&r2r::std_srvs::srv::Empty::Request {})
                    .unwrap()
                    .await
                    .unwrap();
            }
            None => info!("Final nomotion update is not requested!"),
        }
    }
}

impl Localization for Ros2LocalizationClient {
    fn current_pose(&self, _frame_id: &str) -> Result<Isometry2<f64>, Error> {
        let pose_subscriber = self
            .node
            .r2r()
            .subscribe::<PoseWithCovarianceStamped>(
                &self.amcl_pose_topic_name,
                QosProfile::default(),
            )
            .unwrap();

        let subscribed_pose =
            utils::spawn_blocking(
                async move { utils::subscribe_one(pose_subscriber).await.unwrap() },
            )
            .join()
            .unwrap();

        let current_pose = match subscribed_pose {
            Some(msg) => {
                let u_q = na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                    msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                ));

                na::Isometry::from_parts(
                    na::Translation2::new(msg.pose.pose.position.x, msg.pose.pose.position.y),
                    na::UnitComplex::from_angle(u_q.angle()),
                )
            }
            None => {
                return Err(Error::Connection {
                    message: format!("Failed to get pose from {}", self.amcl_pose_topic_name),
                });
            }
        };

        Ok(current_pose)
    }
}

/// Configuration for `Ros2LocalizationClient`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ros2LocalizationClientConfig {
    /// Request final nomotion update hack.
    pub request_final_nomotion_update_hack: bool,
    /// Service name for std_srvs/Empty.
    pub nomotion_update_service_name: String,
    /// Topic name for geometry_msgs/PoseWithCovarianceStamped.
    pub amcl_pose_topic_name: String,
}
