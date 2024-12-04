use std::{sync::Arc, time};

use arci::*;
use nalgebra as na;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::{define_action_client, msg, ActionResultWait};
define_action_client!(MoveBaseActionClient, msg::move_base_msgs, MoveBase);

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
    move_base_action_base_name: String,
    clear_costmap_service_name: String,
    nomotion_update_service_name: String,
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
            move_base_action_base_name: MOVE_BASE_ACTION.to_string(),
            clear_costmap_service_name: CLEAR_COSTMAP_SERVICE.to_string(),
            nomotion_update_service_name: NO_MOTION_UPDATE_SERVICE.to_string(),
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
        let mut c = RosNavClient::new(
            self.request_final_nomotion_update_hack,
            self.move_base_action_base_name,
            self.nomotion_update_service_name,
            self.clear_costmap_service_name,
        );
        c.clear_costmap_before_start = self.clear_costmap_before_start;
        c
    }
}

impl Default for RosNavClientBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone)]
pub struct RosNavClient {
    pub clear_costmap_before_start: bool,
    action_client: Arc<MoveBaseActionClient>,
    nomotion_update_client: Option<rosrust::Client<msg::std_srvs::Empty>>,
    nomotion_update_service_name: String,
    clear_costmap_service_name: String,
}

impl RosNavClient {
    pub fn new(
        request_final_nomotion_update_hack: bool,
        move_base_action_base_name: String,
        nomotion_update_service_name: String,
        clear_costmap_service_name: String,
    ) -> Self {
        let action_client = MoveBaseActionClient::new(&move_base_action_base_name, 1);

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
            clear_costmap_before_start: false,
            action_client: Arc::new(action_client),
            nomotion_update_client,
            nomotion_update_service_name,
            clear_costmap_service_name,
        }
    }

    pub fn new_from_config(config: RosNavClientConfig) -> Self {
        let mut s = Self::new(
            config.request_final_nomotion_update_hack,
            config.move_base_action_base_name,
            config.nomotion_update_service_name,
            config.clear_costmap_service_name,
        );
        s.clear_costmap_before_start = config.clear_costmap_before_start;
        s
    }

    pub fn clear_costmap(&self) -> Result<(), Error> {
        // Wait ten seconds for the service to appear
        rosrust::wait_for_service(
            &self.clear_costmap_service_name,
            Some(time::Duration::from_secs(10)),
        )
        .unwrap();
        // Create a client and call service
        let client =
            rosrust::client::<msg::std_srvs::Empty>(&self.clear_costmap_service_name).unwrap();
        client.req(&msg::std_srvs::EmptyReq {}).unwrap().unwrap();
        rosrust::ros_info!("requested to clear costmaps");
        Ok(())
    }

    fn wait_until_reach(
        &self,
        mut action_result_wait: ActionResultWait,
        timeout: std::time::Duration,
    ) -> Result<(), crate::Error> {
        match action_result_wait.wait(timeout) {
            Ok(_) => {
                rosrust::ros_info!("Action succeeds");
                if let Some(client) = self.nomotion_update_client.as_ref() {
                    client.req(&msg::std_srvs::EmptyReq {}).unwrap().unwrap();
                    rosrust::ros_info!("Called {}", self.nomotion_update_service_name);
                }
                Ok(())
            }
            Err(e) => match e {
                crate::Error::ActionResultPreempted(_) => {
                    rosrust::ros_warn!("Action is cancelled");
                    Err(e)
                }
                _ => {
                    rosrust::ros_err!("Action does not succeed {e:?}");
                    Err(e)
                }
            },
        }
    }
}

impl Navigation for RosNavClient {
    fn send_goal_pose(
        &self,
        goal: na::Isometry2<f64>,
        frame_id: &str,
        timeout: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        if self.clear_costmap_before_start {
            self.clear_costmap()?;
        }
        let mut target_pose = msg::geometry_msgs::PoseStamped {
            pose: goal.into(),
            ..Default::default()
        };
        frame_id.clone_into(&mut target_pose.header.frame_id);
        target_pose.header.stamp = rosrust::now();
        let action_result_wait = self
            .action_client
            .send_goal(msg::move_base_msgs::MoveBaseGoal { target_pose })
            .map_err(|e| anyhow::anyhow!("Failed to send_goal_and_wait : {e}"))?;

        let self_clone = self.clone();
        // Creates a WaitFuture that waits until reach only if the future
        // is polled. This future is a bit tricky, but it's more efficient than
        // using only `tokio::task::spawn_blocking` because it doesn't spawn a thread
        // if the WaitFuture is ignored.
        let wait = WaitFuture::new(async move {
            tokio::task::spawn_blocking(move || {
                self_clone.wait_until_reach(action_result_wait, timeout)
            })
            .await
            .map_err(|e| arci::Error::Other(e.into()))??;
            Ok(())
        });

        Ok(wait)
    }

    fn cancel(&self) -> Result<(), Error> {
        self.action_client
            .cancel_all_goal()
            .map_err(|e| anyhow::anyhow!("Failed to cancel_all_goal : {e}"))?;
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RosNavClientConfig {
    pub request_final_nomotion_update_hack: bool,
    pub clear_costmap_before_start: bool,
    #[serde(default = "default_move_base_action_base_name")]
    pub move_base_action_base_name: String,
    #[serde(default = "default_nomotion_update_service_name")]
    pub nomotion_update_service_name: String,
    #[serde(default = "default_clear_costmap_service_name")]
    pub clear_costmap_service_name: String,
}

fn default_move_base_action_base_name() -> String {
    MOVE_BASE_ACTION.to_string()
}

fn default_nomotion_update_service_name() -> String {
    NO_MOTION_UPDATE_SERVICE.to_string()
}

fn default_clear_costmap_service_name() -> String {
    CLEAR_COSTMAP_SERVICE.to_string()
}
