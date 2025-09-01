use std::{
    sync::{Arc, LazyLock, Mutex},
    time::Duration,
};

use arci::{
    CompleteCondition, JointTrajectoryClient, SetCompleteCondition, TotalJointDiffCondition,
    TrajectoryPoint, WaitFuture,
};
use msg::trajectory_msgs::JointTrajectory;

use crate::{
    JointStateProvider, JointStateProviderFromJointTrajectoryControllerState,
    LazyJointStateProvider, SubscriberHandler,
    create_joint_trajectory_message_for_send_joint_positions,
    create_joint_trajectory_message_for_send_joint_trajectory,
    extract_current_joint_positions_from_state, msg,
};

#[derive(Clone)]
pub struct RosControlClient(Arc<RosControlClientInner>);

struct RosControlClientInner {
    joint_names: Vec<String>,
    send_partial_joints_goal: bool,
    joint_state_provider: Arc<LazyJointStateProvider>,
    complete_condition: Mutex<Arc<dyn CompleteCondition>>,
    trajectory_publisher: rosrust::Publisher<JointTrajectory>,
}

impl RosControlClient {
    pub fn new_with_joint_state_provider(
        joint_names: Vec<String>,
        controller_name: &str,
        send_partial_joints_goal: bool,
        joint_state_provider: Arc<LazyJointStateProvider>,
    ) -> Self {
        let (state_joint_names, _) = joint_state_provider.get_joint_state().unwrap();
        for joint_name in &joint_names {
            if !state_joint_names.iter().any(|name| **name == *joint_name) {
                panic!(
                    "Invalid configuration : Joint ({joint_name}) is not found in state ({state_joint_names:?})",
                );
            }
        }

        let trajectory_publisher =
            rosrust::publish(&format!("{controller_name}/command"), 1).unwrap();

        let rate = rosrust::rate(10.0);
        while rosrust::is_ok() && trajectory_publisher.subscriber_count() == 0 {
            rosrust::ros_info!("waiting trajectory subscriber");
            rate.sleep();
        }

        Self(Arc::new(RosControlClientInner {
            joint_names,
            trajectory_publisher,
            send_partial_joints_goal,
            joint_state_provider,
            complete_condition: Mutex::new(Arc::new(TotalJointDiffCondition::default())),
        }))
    }

    pub fn new(
        joint_names: Vec<String>,
        controller_name: &str,
        send_partial_joints_goal: bool,
    ) -> Self {
        let joint_state_topic_name = Self::state_topic_name(controller_name);
        Self::new_with_state_topic_name(
            joint_names,
            controller_name,
            &joint_state_topic_name,
            send_partial_joints_goal,
        )
    }

    pub(crate) fn state_topic_name(controller_name: &str) -> String {
        format!("{controller_name}/state")
    }

    pub(crate) fn create_joint_state_provider(
        joint_state_topic_name: impl Into<String>,
    ) -> Arc<LazyJointStateProvider> {
        let joint_state_topic_name = joint_state_topic_name.into();
        Arc::new(LazyLock::new(Box::new(move || {
            Box::new(JointStateProviderFromJointTrajectoryControllerState::new(
                SubscriberHandler::new(&joint_state_topic_name, 1),
            ))
        })))
    }

    pub fn new_with_state_topic_name(
        joint_names: Vec<String>,
        controller_name: &str,
        joint_state_topic_name: &str,
        send_partial_joints_goal: bool,
    ) -> Self {
        let joint_state_provider = Self::create_joint_state_provider(joint_state_topic_name);
        Self::new_with_joint_state_provider(
            joint_names,
            controller_name,
            send_partial_joints_goal,
            joint_state_provider,
        )
    }
}

impl JointStateProvider for RosControlClient {
    fn get_joint_state(&self) -> Result<(Vec<String>, Vec<f64>), arci::Error> {
        self.0.joint_state_provider.get_joint_state()
    }
}

impl JointTrajectoryClient for RosControlClient {
    fn joint_names(&self) -> Vec<String> {
        self.0.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        extract_current_joint_positions_from_state(self, self)
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture, arci::Error> {
        let traj = create_joint_trajectory_message_for_send_joint_positions(
            self,
            self,
            &positions,
            duration,
            self.0.send_partial_joints_goal,
        )?;
        self.0.trajectory_publisher.send(traj).unwrap();
        let this = self.clone();
        Ok(WaitFuture::new(async move {
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
            self,
            &trajectory,
            self.0.send_partial_joints_goal,
        )?;
        self.0.trajectory_publisher.send(traj).unwrap();
        let this = self.clone();
        Ok(WaitFuture::new(async move {
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

impl SetCompleteCondition for RosControlClient {
    fn set_complete_condition(&mut self, condition: Box<dyn CompleteCondition>) {
        *self.0.complete_condition.lock().unwrap() = condition.into();
    }
}
