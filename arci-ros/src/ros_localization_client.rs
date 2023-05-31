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
#[serde(deny_unknown_fields)]
pub struct RosLocalizationClientConfig {
    pub request_final_nomotion_update_hack: bool,
    #[serde(default = "default_nomotion_update_service_name")]
    pub nomotion_update_service_name: String,
    #[serde(default = "default_amcl_pose_topic_name")]
    pub amcl_pose_topic_name: String,
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
    amcl_pose_topic_name: String,
    nomotion_update_service_name: String,
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
            amcl_pose_topic_name: AMCL_POSE_TOPIC.to_string(),
            nomotion_update_service_name: NO_MOTION_UPDATE_SERVICE.to_string(),
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
        RosLocalizationClient::new(
            self.request_final_nomotion_update_hack,
            self.nomotion_update_service_name,
            self.amcl_pose_topic_name,
        )
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
    amcl_pose_topic_name: String,
}

impl RosLocalizationClient {
    pub fn new(
        request_final_nomotion_update_hack: bool,
        nomotion_update_service_name: String,
        amcl_pose_topic_name: String,
    ) -> Self {
        let pose_subscriber = SubscriberHandler::new(&amcl_pose_topic_name, 1);
        let nomotion_update_client = if request_final_nomotion_update_hack {
            rosrust::wait_for_service(
                &nomotion_update_service_name,
                Some(std::time::Duration::from_secs(10)),
            )
            .unwrap();
            Some(rosrust::client::<msg::std_srvs::Empty>(&nomotion_update_service_name).unwrap())
        } else {
            None
        };
        Self {
            pose_subscriber,
            nomotion_update_client,
            amcl_pose_topic_name,
        }
    }

    pub fn new_from_config(config: RosLocalizationClientConfig) -> Self {
        Self::new(
            config.request_final_nomotion_update_hack,
            config.nomotion_update_service_name,
            config.amcl_pose_topic_name,
        )
    }

    pub fn request_nomotion_update(&self) {
        self.nomotion_update_client
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
                    message: format!("Failed to get pose from {}", self.amcl_pose_topic_name),
                })?;
        let pose: na::Isometry3<f64> = pose_with_cov_stamped.pose.pose.into();

        Ok(na::Isometry2::new(
            na::Vector2::new(pose.translation.vector[0], pose.translation.vector[1]),
            pose.rotation.euler_angles().2,
        ))
    }
}

fn default_nomotion_update_service_name() -> String {
    NO_MOTION_UPDATE_SERVICE.to_string()
}

fn default_amcl_pose_topic_name() -> String {
    AMCL_POSE_TOPIC.to_string()
}
