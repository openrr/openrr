use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

use arci::{nalgebra as na, *};
use futures::StreamExt;
use parking_lot::Mutex;
use r2r::{geometry_msgs::msg::PoseWithCovarianceStamped, QosProfile};
use serde::{Deserialize, Serialize};
use tracing::info;

use crate::utils;

/// Implement arci::Localization for ROS2
pub struct Ros2LocalizationClient {
    node: Arc<Mutex<r2r::Node>>,
    nomotion_update_client: Option<r2r::Client<r2r::std_srvs::srv::Empty::Service>>,
    amcl_pose_topic_name: String,
}

impl Ros2LocalizationClient {
    /// Creates a new `Ros2LocalizationClient`.
    pub fn new(
        ctx: r2r::Context,
        request_final_nomotion_update_hack: bool,
        nomotion_update_service_name: &str,
        amcl_pose_topic_name: &str,
    ) -> Self {
        let node = r2r::Node::create(ctx, "openrr_ros2_localization_node", "arci_ros2").unwrap();

        Self::from_node(
            node,
            request_final_nomotion_update_hack,
            nomotion_update_service_name,
            amcl_pose_topic_name,
        )
    }

    /// Creates a new `Ros2LocalizationClient` from ROS2 node.
    pub fn from_node(
        mut node: r2r::Node,
        request_final_nomotion_update_hack: bool,
        nomotion_update_service_name: &str,
        amcl_pose_topic_name: &str,
    ) -> Self {
        let nomotion_update_client = if request_final_nomotion_update_hack {
            Some(node.create_client(&nomotion_update_service_name).unwrap())
        } else {
            None
        };

        Self {
            node: Arc::new(Mutex::new(node)),
            nomotion_update_client,
            amcl_pose_topic_name: amcl_pose_topic_name.to_owned(),
        }
    }

    /// Request final nomotion update hack
    pub async fn request_nomotion_update(&self) {
        match self.nomotion_update_client.as_ref() {
            Some(client) => {
                self.node
                    .lock()
                    .is_available(client)
                    .unwrap()
                    .await
                    .unwrap();

                client
                    .request(&r2r::std_srvs::srv::Empty::Request {})
                    .unwrap()
                    .await
                    .unwrap();
            }
            None => info!("Final nomotion update is not requested!"),
        }
    }

    /// TODO:
    pub fn spin_node_once(&self, timeout: std::time::Duration) {
        self.node.lock().spin_once(timeout);
    }
}

impl Localization for Ros2LocalizationClient {
    fn current_pose(&self, _frame_id: &str) -> Result<Isometry2<f64>, Error> {
        let node_clone = self.node.clone();

        let mut pose_subscriber = node_clone
            .lock()
            .subscribe::<PoseWithCovarianceStamped>(
                &self.amcl_pose_topic_name,
                QosProfile::default(),
            )
            .unwrap();

        let is_done = Arc::new(AtomicBool::new(false));
        let is_done_clone = is_done.clone();

        let subscribed_pose = utils::spawn_blocking(async move {
            let handle = tokio::spawn(async move {
                let next = pose_subscriber.next().await;
                is_done_clone.store(true, Ordering::SeqCst);
                next
            });
            loop {
                if is_done.load(Ordering::SeqCst) {
                    break;
                }
                node_clone.lock().spin_once(Duration::from_millis(100));
                tokio::task::yield_now().await;
            }
            handle.await.unwrap()
        })
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

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
/// Configuration for Ros2LocalizationClientConfig
pub struct Ros2LocalizationClientConfig {
    /// request final nomotion update hack
    pub request_final_nomotion_update_hack: bool,
    /// name of service std_srvs/Empty
    pub nomotion_update_service_name: String,
    /// topic name for PoseWithCovarianceStamped
    pub amcl_pose_topic_name: String,
}
