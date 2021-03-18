use crate::msg;
use arci::*;
use nalgebra as na;
use serde::{Deserialize, Serialize};
use std::time;

use crate::define_action_client_internal;
define_action_client_internal!(SimpleActionClient, msg::move_base_msgs, MoveBase);

rosrust::rosmsg_include! {
    std_srvs / Empty
}

impl From<na::Isometry2<f64>> for msg::geometry_msgs::Pose {
    fn from(goal: na::Isometry2<f64>) -> Self {
        let iso_pose = na::Isometry3::from_parts(
            na::Translation3::new(goal.translation.x, goal.translation.y, 0.0),
            na::UnitQuaternion::from_euler_angles(0.0, 0.0, goal.rotation.angle()),
        );
        iso_pose.into()
    }
}

const NO_MOTION_UPDATE_SERVICE: &str = "request_nomotion_update";
const MOVE_BASE_ACTION: &str = "/move_base";
const CLEAR_COSTMAP_SERVICE: &str = "/move_base/clear_costmaps";

/// Build RosNavClient interactively.
///
/// # Examples
///
/// ```no_run
/// let client = arci_ros::RosNavClientBuilder::new().clear_costmap_before_start(true).finalize();
/// ```
#[derive(Clone, Debug)]
pub struct RosNavClientBuilder {
    /* TODO
    move_base_action_name: String,
    amcl_pose_topic_name: String,
    no_motion_update_service: String,
    */
    request_final_nomotion_update_hack: bool,
    clear_costmap_before_start: bool,
}

impl RosNavClientBuilder {
    /// Create builder
    ///
    /// # Examples
    ///
    /// ```
    /// let builder = arci_ros::RosNavClientBuilder::new();
    /// ```
    pub fn new() -> Self {
        Self {
            /* TODO:
            move_base_action_name: MOVE_BASE_ACTION.to_string(),
            amcl_pose_topic_name: AMCL_POSE_TOPIC.to_string(),
            no_motion_update_service: NO_MOTION_UPDATE_SERVICE.to_string(),
            */
            request_final_nomotion_update_hack: false,
            clear_costmap_before_start: false,
        }
    }
    /// Enable/Disable request_final_nomotion_update_hack
    ///
    /// # Examples
    ///
    /// ```
    /// // true means enable (default: false)
    /// let builder = arci_ros::RosNavClientBuilder::new().request_final_nomotion_update_hack(true);
    /// ```
    pub fn request_final_nomotion_update_hack(mut self, val: bool) -> Self {
        self.request_final_nomotion_update_hack = val;
        self
    }
    /// Enable/Disable clear_costmap_before_start
    ///
    /// # Examples
    ///
    /// ```
    /// // true means enable (default: false)
    /// let builder = arci_ros::RosNavClientBuilder::new().clear_costmap_before_start(true);
    /// ```

    pub fn clear_costmap_before_start(mut self, val: bool) -> Self {
        self.clear_costmap_before_start = val;
        self
    }
    /// Convert builder into RosNavClient finally.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// let client = arci_ros::RosNavClientBuilder::new().finalize();
    /// ```
    pub fn finalize(self) -> RosNavClient {
        let mut c = RosNavClient::new(self.request_final_nomotion_update_hack);
        c.clear_costmap_before_start = self.clear_costmap_before_start;
        c
    }
}

impl Default for RosNavClientBuilder {
    fn default() -> Self {
        Self::new()
    }
}

pub struct RosNavClient {
    pub clear_costmap_before_start: bool,
    action_client: SimpleActionClient,
    nomotion_update_client: Option<rosrust::Client<std_srvs::Empty>>,
}

impl RosNavClient {
    pub fn new(request_final_nomotion_update_hack: bool) -> Self {
        let action_client = SimpleActionClient::new(MOVE_BASE_ACTION, 1, 10.0);

        let nomotion_update_client = if request_final_nomotion_update_hack {
            rosrust::wait_for_service(
                NO_MOTION_UPDATE_SERVICE,
                Some(std::time::Duration::from_secs(10)),
            )
            .unwrap();
            Some(rosrust::client::<std_srvs::Empty>(NO_MOTION_UPDATE_SERVICE).unwrap())
        } else {
            None
        };
        Self {
            clear_costmap_before_start: false,
            action_client,
            nomotion_update_client,
        }
    }
    pub fn new_from_config(config: RosNavClientConfig) -> Self {
        let mut s = Self::new(config.request_final_nomotion_update_hack);
        s.clear_costmap_before_start = config.clear_costmap_before_start;
        s
    }
    pub fn clear_costmap(&self) -> Result<(), Error> {
        // Wait ten seconds for the service to appear
        rosrust::wait_for_service(CLEAR_COSTMAP_SERVICE, Some(time::Duration::from_secs(10)))
            .unwrap();
        // Create a client and call service
        let client = rosrust::client::<msg::std_srvs::Empty>(CLEAR_COSTMAP_SERVICE).unwrap();
        client.req(&msg::std_srvs::EmptyReq {}).unwrap().unwrap();
        rosrust::ros_info!("requested to clear costmaps");
        Ok(())
    }

    fn wait_until_reach(&self, goal_id: &str, timeout: std::time::Duration) {
        match self.action_client.wait_for_result(goal_id, timeout) {
            Ok(_) => {
                rosrust::ros_info!("Action succeeds");
                if let Some(client) = self.nomotion_update_client.as_ref() {
                    client.req(&std_srvs::EmptyReq {}).unwrap().unwrap();
                    rosrust::ros_info!("Called {}", NO_MOTION_UPDATE_SERVICE);
                }
            }
            Err(e) => {
                match e {
                    crate::Error::ActionResultPreempted(_) => {
                        rosrust::ros_warn!("Action is cancelled");
                    }
                    _ => {
                        rosrust::ros_err!("Action does not succeed {:?}", e);
                    }
                };
            }
        };
    }
}

#[async_trait]
impl Navigation for RosNavClient {
    async fn send_pose(
        &self,
        goal: na::Isometry2<f64>,
        frame_id: &str,
        timeout: std::time::Duration,
    ) -> Result<(), Error> {
        if self.clear_costmap_before_start {
            self.clear_costmap()?;
        }
        let mut target_pose = msg::geometry_msgs::PoseStamped {
            pose: goal.into(),
            ..Default::default()
        };
        target_pose.header.frame_id = frame_id.to_owned();
        target_pose.header.stamp = rosrust::now();
        let goal_id = self
            .action_client
            .send_goal(msg::move_base_msgs::MoveBaseGoal { target_pose })
            .map_err(|e| anyhow::anyhow!("Failed to send_goal_and_wait : {}", e.to_string()))?;
        self.wait_until_reach(&goal_id, timeout);
        Ok(())
    }

    fn cancel(&self) -> Result<(), Error> {
        self.action_client
            .cancel_all_goal()
            .map_err(|e| anyhow::anyhow!("Failed to cancel_all_goal : {}", e.to_string()))?;
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct RosNavClientConfig {
    pub request_final_nomotion_update_hack: bool,
    pub clear_costmap_before_start: bool,
}
