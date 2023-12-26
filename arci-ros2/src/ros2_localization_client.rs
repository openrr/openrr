use std::{
    sync::{Arc, RwLock},
    time::Duration,
};

use arci::{nalgebra as na, *};
use serde::{Deserialize, Serialize};
use tracing::info;

use crate::{
    msg::{
        geometry_msgs::{PoseWithCovariance, PoseWithCovarianceStamped},
        std_msgs, std_srvs,
    },
    utils, Node,
};

/// `arci::Localization` implementation for ROS2.
pub struct Ros2LocalizationClient {
    nomotion_update_client: Option<ros2_client::Client<std_srvs::Empty>>,
    pose: Arc<RwLock<Option<PoseWithCovariance>>>,
    amcl_pose_topic: rustdds::Topic,
    // keep not to be dropped
    _node: Node,
}

impl Ros2LocalizationClient {
    /// Creates a new `Ros2LaserScan2D`.
    pub fn new(
        node: Node,
        request_final_nomotion_update_hack: bool,
        nomotion_update_service_name: &str,
        amcl_pose_topic_name: &str,
    ) -> Result<Self, Error> {
        let amcl_pose_topic = node
            .create_topic::<PoseWithCovarianceStamped>(amcl_pose_topic_name)
            .unwrap();
        let mut pose_subscriber =
            node.create_subscription::<PoseWithCovarianceStamped>(&amcl_pose_topic)?;
        let pose = utils::subscribe_one(&mut pose_subscriber, Duration::from_secs(1))
            .map(|pose| pose.pose);
        let pose = Arc::new(RwLock::new(pose));
        utils::subscribe_thread(pose_subscriber, pose.clone(), |pose| Some(pose.pose));

        let nomotion_update_client = if request_final_nomotion_update_hack {
            Some(node.create_client(nomotion_update_service_name)?)
        } else {
            None
        };

        Ok(Self {
            nomotion_update_client,
            pose,
            amcl_pose_topic,
            _node: node,
        })
    }

    /// Request final nomotion update hack
    pub async fn request_nomotion_update(&self) {
        match self.nomotion_update_client.as_ref() {
            Some(client) => {
                // TODO
                // let is_available = self.node.r2r().is_available(client).unwrap();
                // is_available.await.unwrap();

                client.async_send_request(std_msgs::Empty {}).await.unwrap();
            }
            None => info!("Final nomotion update is not requested!"),
        }
    }
}

impl Localization for Ros2LocalizationClient {
    fn current_pose(&self, _frame_id: &str) -> Result<Isometry2<f64>, Error> {
        let subscribed_pose = self.pose.read().unwrap();
        let current_pose = match &*subscribed_pose {
            Some(msg) => {
                let u_q = na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                    msg.pose.orientation.w,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                ));

                na::Isometry::from_parts(
                    na::Translation2::new(msg.pose.position.x, msg.pose.position.y),
                    na::UnitComplex::from_angle(u_q.angle()),
                )
            }
            None => {
                return Err(Error::Connection {
                    message: format!(
                        "Failed to get pose from {}",
                        rustdds::TopicDescription::name(&self.amcl_pose_topic)
                    ),
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
