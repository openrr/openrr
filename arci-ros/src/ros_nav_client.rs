use crate::msg;
use crate::rosrust_utils::*;
use arci::*;
use nalgebra as na;
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

pub struct RosNavClient {
    pose_subscriber: SubscriberHandler<msg::geometry_msgs::PoseWithCovarianceStamped>,
    pub clear_costmap_before_start: bool,
    pub frame_id: String,
    action_client: SimpleActionClient,
    nomotion_update_client: Option<rosrust::Client<std_srvs::Empty>>,
}

impl RosNavClient {
    const AMCL_POSE_TOPIC: &'static str = "/amcl_pose";
    const NO_MOTION_UPDATE_SERVICE: &'static str = "request_nomotion_update";
    pub fn new(request_final_nomotion_update_hack: bool) -> Self {
        let action_client = SimpleActionClient::new("/move_base", 1, 10.0);

        let pose_subscriber = SubscriberHandler::new(Self::AMCL_POSE_TOPIC, 1);
        let nomotion_update_client = if request_final_nomotion_update_hack {
            rosrust::wait_for_service(
                Self::NO_MOTION_UPDATE_SERVICE,
                Some(std::time::Duration::from_secs(10)),
            )
            .unwrap();
            Some(rosrust::client::<std_srvs::Empty>(Self::NO_MOTION_UPDATE_SERVICE).unwrap())
        } else {
            None
        };
        Self {
            pose_subscriber,
            clear_costmap_before_start: false,
            frame_id: "".to_string(),
            action_client,
            nomotion_update_client,
        }
    }
    pub fn clear_costmap(&self) -> Result<(), Error> {
        const CLEAR_COSTMAP_SERVICE: &str = "/move_base/clear_costmaps";
        // Wait ten seconds for the service to appear
        rosrust::wait_for_service(CLEAR_COSTMAP_SERVICE, Some(time::Duration::from_secs(10)))
            .unwrap();
        // Create a client and call service
        let client = rosrust::client::<msg::std_srvs::Empty>(CLEAR_COSTMAP_SERVICE).unwrap();
        client.req(&msg::std_srvs::EmptyReq {}).unwrap().unwrap();
        rosrust::ros_info!("requested to clear costmaps");
        Ok(())
    }

    fn wait_until_reach(&self, goal_id: &str, timeout: std::time::Duration) -> Result<(), Error> {
        match self.action_client.wait_for_result(goal_id, timeout) {
            Ok(_) => {
                rosrust::ros_info!("Action succeeds");
                if let Some(client) = self.nomotion_update_client.as_ref() {
                    client.req(&std_srvs::EmptyReq {}).unwrap().unwrap();
                    rosrust::ros_info!("Called {}", Self::NO_MOTION_UPDATE_SERVICE);
                }
            }
            Err(e) => {
                match e {
                    crate::Error::ActionResultPreempted(_) => {
                        rosrust::ros_info!("Action is cancelled");
                    }
                    _ => {
                        rosrust::ros_info!("Action does not succeed");
                    }
                };
            }
        };
        Ok(())
    }
}

#[async_trait]
impl Navigation for RosNavClient {
    async fn send_pose(
        &self,
        goal: na::Isometry2<f64>,
        timeout: std::time::Duration,
    ) -> Result<(), Error> {
        if self.clear_costmap_before_start {
            self.clear_costmap()?;
        }
        let mut target_pose = msg::geometry_msgs::PoseStamped::default();
        target_pose.pose = goal.into();
        target_pose.header.frame_id = self.frame_id.clone();
        target_pose.header.stamp = rosrust::now();
        let goal_id = self
            .action_client
            .send_goal(msg::move_base_msgs::MoveBaseGoal { target_pose })
            .map_err(|e| anyhow::anyhow!("Failed to send_goal_and_wait : {}", e.to_string()))?;
        Ok(self.wait_until_reach(&goal_id, timeout)?)
    }

    fn cancel(&self) -> Result<(), Error> {
        self.action_client
            .cancel_all_goal()
            .map_err(|e| anyhow::anyhow!("Failed to cancel_all_goal : {}", e.to_string()))?;
        Ok(())
    }

    fn current_pose(&self) -> Result<na::Isometry2<f64>, Error> {
        self.pose_subscriber.wait_message(100);
        let pose_with_cov_stamped =
            self.pose_subscriber
                .get()?
                .ok_or_else(|| Error::Connection {
                    message: format!("Failed to get pose from {}", Self::AMCL_POSE_TOPIC),
                })?;
        let pose: na::Isometry3<f64> = pose_with_cov_stamped.pose.pose.into();

        Ok(na::Isometry2::new(
            na::Vector2::new(pose.translation.vector[0], pose.translation.vector[1]),
            pose.rotation.euler_angles().2,
        ))
    }
}
