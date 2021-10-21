use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use arci::{
    CompleteCondition, JointTrajectoryClient, SetCompleteCondition, TotalJointDiffCondition,
    TrajectoryPoint, WaitFuture,
};
use msg::control_msgs::JointTrajectoryControllerState;

use crate::{
    create_joint_trajectory_message_for_send_joint_positions,
    create_joint_trajectory_message_for_send_joint_trajectory, define_action_client_internal,
    error::Error, extract_current_joint_positions_from_message, msg, rosrust_utils::*,
};

const ACTION_TIMEOUT_DURATION_RATIO: u32 = 10;

define_action_client_internal!(SimpleActionClient, msg::control_msgs, FollowJointTrajectory);

#[derive(Clone)]
pub struct RosControlActionClient(Arc<RosControlActionClientInner>);

struct RosControlActionClientInner {
    joint_names: Vec<String>,
    joint_state_subscriber_handler: SubscriberHandler<JointTrajectoryControllerState>,
    action_client: SimpleActionClient,
    complete_condition: Mutex<Arc<dyn CompleteCondition>>,
}

impl RosControlActionClient {
    pub fn new(joint_names: Vec<String>, controller_name: &str) -> Self {
        let joint_state_topic_name = format!("{}/state", controller_name);
        let joint_state_subscriber_handler = SubscriberHandler::new(&joint_state_topic_name, 1);
        joint_state_subscriber_handler.wait_message(100);
        let action_client =
            SimpleActionClient::new(&format!("{}/follow_joint_trajectory", controller_name), 1);

        Self(Arc::new(RosControlActionClientInner {
            joint_names,
            joint_state_subscriber_handler,
            action_client,
            complete_condition: Mutex::new(Arc::new(TotalJointDiffCondition::default())),
        }))
    }

    pub fn get_joint_state(&self) -> Result<JointTrajectoryControllerState, arci::Error> {
        self.0
            .joint_state_subscriber_handler
            .get()?
            .ok_or_else(|| arci::Error::Other(Error::NoJointStateAvailable.into()))
    }
}

impl JointTrajectoryClient for RosControlActionClient {
    fn joint_names(&self) -> Vec<String> {
        self.0.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        extract_current_joint_positions_from_message(self, self.get_joint_state()?)
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture, arci::Error> {
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
        let mut goal_id = self
            .0
            .action_client
            .send_goal(goal)
            .map_err(|e| anyhow::anyhow!(e.to_string()))?;
        let this = self.clone();
        Ok(WaitFuture::new(async move {
            // Success of action result does not always mean joints reached to target.
            // So check complete_condition too.
            tokio::task::spawn_blocking(move || {
                goal_id.wait(duration * ACTION_TIMEOUT_DURATION_RATIO)
            })
            .await
            .map_err(|e| arci::Error::Other(e.into()))??;
            // Clone to avoid holding the lock for a long time.
            let complete_condition = this.0.complete_condition.lock().unwrap().clone();
            complete_condition
                .wait(&this, &positions, duration.as_secs_f64())
                .await
        }))
    }

    fn send_joint_trajectory(
        &self,
        trajectory: Vec<TrajectoryPoint>,
    ) -> Result<WaitFuture, arci::Error> {
        let traj = create_joint_trajectory_message_for_send_joint_trajectory(
            self,
            self.get_joint_state()?,
            &trajectory,
        )?;
        let goal = msg::control_msgs::FollowJointTrajectoryGoal {
            trajectory: traj,
            ..Default::default()
        };
        let mut goal_id = self
            .0
            .action_client
            .send_goal(goal)
            .map_err(|e| anyhow::anyhow!(e.to_string()))?;
        let this = self.clone();
        Ok(WaitFuture::new(async move {
            let duration = if let Some(trajectory_point) = trajectory.last() {
                trajectory_point.time_from_start
            } else {
                Duration::from_secs(0)
            };
            // Success of action result does not always mean joints reached to target.
            // So check complete_condition too.
            tokio::task::spawn_blocking(move || {
                goal_id.wait(duration * ACTION_TIMEOUT_DURATION_RATIO)
            })
            .await
            .map_err(|e| arci::Error::Other(e.into()))??;
            // Clone to avoid holding the lock for a long time.
            let complete_condition = this.0.complete_condition.lock().unwrap().clone();
            complete_condition
                .wait(
                    &this,
                    &trajectory.last().unwrap().positions,
                    trajectory.last().unwrap().time_from_start.as_secs_f64(),
                )
                .await
        }))
    }
}

impl SetCompleteCondition for RosControlActionClient {
    fn set_complete_condition(&mut self, condition: Box<dyn CompleteCondition>) {
        *self.0.complete_condition.lock().unwrap() = condition.into();
    }
}
