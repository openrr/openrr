use std::borrow::Borrow;

use arci::*;
use nalgebra as na;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::{msg, rosrust_utils::*};

rosrust::rosmsg_include! {
    std_srvs / Empty
}

const AMCL_POSE_TOPIC: &str = "/amcl_pose";
const NO_MOTION_UPDATE_SERVICE: &str = "request_nomotion_update";

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
pub struct RosLocalizationClientConfig {
    pub request_final_nomotion_update_hack: bool,
}

/// Build RosLocalizationClient interactively.
///
/// # Examples
///
/// ```no_run
/// let client = arci_ros::RosLocalizationClientBuilder::new().request_final_nomotion_update_hack(true).finalize();
/// ```
#[derive(Clone, Debug)]
pub struct RosLocalizationClientBuilder {
    /* TODO
    amcl_pose_topic_name: String,
    no_motion_update_service: String,
    */
    request_final_nomotion_update_hack: bool,
}

impl RosLocalizationClientBuilder {
    /// Create builder
    ///
    /// # Examples
    ///
    /// ```
    /// let builder = arci_ros::RosLocalizationClientBuilder::new();
    /// ```
    pub fn new() -> Self {
        Self {
            /* TODO:
            amcl_pose_topic_name: AMCL_POSE_TOPIC.to_string(),
            no_motion_update_service: NO_MOTION_UPDATE_SERVICE.to_string(),
            */
            request_final_nomotion_update_hack: false,
        }
    }

    /// Enable/Disable request_final_nomotion_update_hack
    ///
    /// # Examples
    ///
    /// ```
    /// // true means enable (default: false)
    /// let builder = arci_ros::RosLocalizationClientBuilder::new().request_final_nomotion_update_hack(true);
    /// ```
    pub fn request_final_nomotion_update_hack(mut self, val: bool) -> Self {
        self.request_final_nomotion_update_hack = val;
        self
    }

    /// Convert builder into RosLocalizationClient finally.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// let client = arci_ros::RosLocalizationClientBuilder::new().finalize();
    /// ```
    pub fn finalize(self) -> RosLocalizationClient {
        RosLocalizationClient::new(self.request_final_nomotion_update_hack)
    }
}

impl Default for RosLocalizationClientBuilder {
    fn default() -> Self {
        Self::new()
    }
}

pub struct RosLocalizationClient {
    pose_subscriber: SubscriberHandler<msg::geometry_msgs::PoseWithCovarianceStamped>,
    nomotion_update_client: Option<rosrust::Client<msg::std_srvs::Empty>>,
}

impl RosLocalizationClient {
    pub fn new(request_final_nomotion_update_hack: bool) -> Self {
        let pose_subscriber = SubscriberHandler::new(AMCL_POSE_TOPIC, 1);
        let nomotion_update_client = if request_final_nomotion_update_hack {
            rosrust::wait_for_service(
                NO_MOTION_UPDATE_SERVICE,
                Some(std::time::Duration::from_secs(10)),
            )
            .unwrap();
            Some(rosrust::client::<msg::std_srvs::Empty>(NO_MOTION_UPDATE_SERVICE).unwrap())
        } else {
            None
        };
        Self {
            pose_subscriber,
            nomotion_update_client,
        }
    }

    pub fn new_from_config(config: RosLocalizationClientConfig) -> Self {
        Self::new(config.request_final_nomotion_update_hack)
    }

    pub fn request_nomotion_update(&self) {
        self.nomotion_update_client
            .borrow()
            .as_ref()
            .unwrap()
            .req(&msg::std_srvs::EmptyReq {})
            .unwrap()
            .unwrap();
    }
}

impl Localization for RosLocalizationClient {
    fn current_pose(&self, _frame_id: &str) -> Result<na::Isometry2<f64>, Error> {
        self.pose_subscriber.wait_message(100);
        let pose_with_cov_stamped =
            self.pose_subscriber
                .get()?
                .ok_or_else(|| Error::Connection {
                    message: format!("Failed to get pose from {}", AMCL_POSE_TOPIC),
                })?;
        let pose: na::Isometry3<f64> = pose_with_cov_stamped.pose.pose.into();

        Ok(na::Isometry2::new(
            na::Vector2::new(pose.translation.vector[0], pose.translation.vector[1]),
            pose.rotation.euler_angles().2,
        ))
    }
}
