use crate::{
    create_joint_trajectory_message_for_send_joint_positions,
    create_joint_trajectory_message_for_send_joint_trajectory, define_action_client_internal,
    error::Error, extract_current_joint_positions_from_message, msg, rosrust_utils::*,
};

use arci::{
    CompleteCondition, JointTrajectoryClient, SetCompleteCondition, TotalJointDiffCondition,
    TrajectoryPoint,
};
use async_trait::async_trait;

use msg::control_msgs::JointTrajectoryControllerState;

define_action_client_internal!(SimpleActionClient, msg::control_msgs, FollowJointTrajectory);

pub struct RosControlActionClient {
    joint_names: Vec<String>,
    joint_state_subscriber_handler: SubscriberHandler<JointTrajectoryControllerState>,
    action_client: SimpleActionClient,
    complete_condition: Box<dyn CompleteCondition>,
}

impl RosControlActionClient {
    pub fn new(joint_names: Vec<String>, controller_name: &str) -> Self {
        let joint_state_topic_name = format!("{}/state", controller_name);
        let joint_state_subscriber_handler = SubscriberHandler::new(&joint_state_topic_name, 1);
        joint_state_subscriber_handler.wait_message(100);
        let action_client = SimpleActionClient::new(
            &format!("{}/follow_joint_trajectory", controller_name),
            1,
            10.0,
        );

        Self {
            joint_names,
            joint_state_subscriber_handler,
            action_client,
            complete_condition: Box::new(TotalJointDiffCondition::default()),
        }
    }
    pub fn get_joint_state(&self) -> Result<JointTrajectoryControllerState, arci::Error> {
        self.joint_state_subscriber_handler
            .get()?
            .ok_or_else(|| arci::Error::Other(Error::NoJointStateAvailable.into()))
    }
}

#[async_trait]
impl JointTrajectoryClient for RosControlActionClient {
    fn joint_names(&self) -> &[String] {
        &self.joint_names
    }
    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        Ok(extract_current_joint_positions_from_message(
            self,
            self.get_joint_state()?,
        ))
    }

    async fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<(), arci::Error> {
        let traj = create_joint_trajectory_message_for_send_joint_positions(
            self,
            self.get_joint_state()?,
            &positions,
            duration,
        )?;

        let goal = msg::control_msgs::FollowJointTrajectoryGoal {
            trajectory: traj,
            ..Default::default()
        };
        let _goal_id = self
            .action_client
            .send_goal(goal)
            .map_err(|e| anyhow::anyhow!(e.to_string()))?;
        self.complete_condition.wait(self, &positions, None)
        /*
        // TODO use action result
        Ok(SimpleActionClientWait::new_boxed(
            &self.action_client,
            goal_id,
            duration * 10,
        ))
        */
    }
    async fn send_joint_trajectory(
        &self,
        trajectory: Vec<TrajectoryPoint>,
    ) -> Result<(), arci::Error> {
        let traj = create_joint_trajectory_message_for_send_joint_trajectory(
            self,
            self.get_joint_state()?,
            &trajectory,
        )?;
        let goal = msg::control_msgs::FollowJointTrajectoryGoal {
            trajectory: traj,
            ..Default::default()
        };
        let _goal_id = self
            .action_client
            .send_goal(goal)
            .map_err(|e| anyhow::anyhow!(e.to_string()))?;
        self.complete_condition
            .wait(self, &trajectory.last().unwrap().positions, None)
        /*
        // TODO use action result
        let duration = if let Some(trajectory_point) = trajectory.last() {
            trajectory_point.time_from_start
        } else {
            std::time::Duration::from_secs(0)
        };
        Ok(SimpleActionClientWait::new_boxed(
            &self.action_client,
            goal_id,
            duration * 10,
        ))
        */
    }
}

impl SetCompleteCondition for RosControlActionClient {
    fn set_complete_condition(&mut self, condition: Box<dyn CompleteCondition>) {
        self.complete_condition = condition;
    }
}
