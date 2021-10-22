use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use arci::{
    CompleteCondition, EachJointDiffCondition, JointTrajectoryClient, SetCompleteCondition,
    TotalJointDiffCondition, TrajectoryPoint, WaitFuture,
};
use msg::sensor_msgs::JointState;
use once_cell::sync::Lazy;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::{
    define_action_client_internal,
    error::Error,
    msg,
    ros_control_common::{
        create_joint_trajectory_message_for_send_joint_positions,
        create_joint_trajectory_message_for_send_joint_trajectory,
        extract_current_joint_positions_from_state, JointStateProvider, LazyJointStateProvider,
    },
    JointTrajectoryClientWrapperConfig, RosControlClientBuilder, SubscriberHandler,
};

const ACTION_TIMEOUT_DURATION_RATIO: u32 = 10;

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RosControlActionClientConfig {
    pub name: String,
    pub joint_names: Vec<String>,

    pub controller_name: String,
    pub state_topic_name: String,
    #[serde(default)]
    pub send_partial_joints_goal: bool,
    pub complete_allowable_errors: Vec<f64>,
    #[serde(default = "default_complete_timeout_sec")]
    pub complete_timeout_sec: f64,

    // TOML format has a restriction that if a table itself contains tables,
    // all keys with non-table values must be emitted first.
    // Therefore, these fields must be located at the end of the struct.
    #[serde(flatten)]
    pub wrapper_config: JointTrajectoryClientWrapperConfig,
}

const fn default_complete_timeout_sec() -> f64 {
    10.0
}

impl RosControlClientBuilder for RosControlActionClientConfig {
    fn build_joint_state_provider(
        &self,
        joint_state_topic_name: impl Into<String>,
    ) -> Arc<LazyJointStateProvider> {
        RosControlActionClient::create_joint_state_provider(joint_state_topic_name)
    }

    fn build_joint_trajectory_client(
        &self,
        lazy: bool,
        joint_state_provider: Arc<LazyJointStateProvider>,
    ) -> Result<Arc<dyn JointTrajectoryClient>, arci::Error> {
        let config = self.clone();
        let create_client = move || {
            let RosControlActionClientConfig {
                joint_names,
                controller_name,
                send_partial_joints_goal,
                complete_allowable_errors,
                complete_timeout_sec,
                ..
            } = config;

            rosrust::ros_debug!(
                "create_joint_trajectory_clients_inner: creating RosControlActionClient"
            );
            let mut client = RosControlActionClient::new_with_joint_state_provider(
                joint_names,
                &controller_name,
                send_partial_joints_goal,
                joint_state_provider,
            );
            client.set_complete_condition(Box::new(EachJointDiffCondition::new(
                complete_allowable_errors,
                complete_timeout_sec,
            )));
            Ok(client)
        };
        Ok(if lazy {
            Arc::new(arci::Lazy::with_joint_names(
                create_client,
                self.joint_names.clone(),
            ))
        } else {
            Arc::new(create_client().unwrap())
        })
    }

    fn state_topic(&self) -> String {
        self.state_topic_name.clone()
    }

    fn wrapper_config(&self) -> &JointTrajectoryClientWrapperConfig {
        &self.wrapper_config
    }

    fn name(&self) -> &str {
        &self.name
    }
}

struct JointStateProviderFromJointState(SubscriberHandler<JointState>);

impl JointStateProviderFromJointState {
    fn new(subscriber_handler: SubscriberHandler<JointState>) -> Self {
        subscriber_handler.wait_message(100);
        Self(subscriber_handler)
    }
}

impl JointStateProvider for JointStateProviderFromJointState {
    fn get_joint_state(&self) -> Result<(Vec<String>, Vec<f64>), arci::Error> {
        let state = self
            .0
            .get()?
            .ok_or_else(|| arci::Error::Other(Error::NoJointStateAvailable.into()))?;
        Ok((state.name, state.position))
    }
}

define_action_client_internal!(SimpleActionClient, msg::control_msgs, FollowJointTrajectory);

#[derive(Clone)]
pub struct RosControlActionClient(Arc<RosControlActionClientInner>);

struct RosControlActionClientInner {
    joint_names: Vec<String>,
    send_partial_joints_goal: bool,
    joint_state_provider: Arc<LazyJointStateProvider>,
    complete_condition: Mutex<Arc<dyn CompleteCondition>>,
    action_client: SimpleActionClient,
}

impl RosControlActionClient {
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
                    "Invalid configuration : Joint ({}) is not found in state ({:?})",
                    joint_name, state_joint_names
                );
            }
        }

        let action_client =
            SimpleActionClient::new(&format!("{}/follow_joint_trajectory", controller_name), 1);

        Self(Arc::new(RosControlActionClientInner {
            joint_names,
            joint_state_provider,
            send_partial_joints_goal,
            action_client,
            complete_condition: Mutex::new(Arc::new(TotalJointDiffCondition::default())),
        }))
    }

    pub fn new(
        joint_names: Vec<String>,
        controller_name: &str,
        send_partial_joints_goal: bool,
        joint_state_topic_name: &str,
    ) -> Self {
        Self::new_with_joint_state_provider(
            joint_names,
            controller_name,
            send_partial_joints_goal,
            Self::create_joint_state_provider(joint_state_topic_name),
        )
    }

    fn create_joint_state_provider(
        joint_state_topic_name: impl Into<String>,
    ) -> Arc<LazyJointStateProvider> {
        let joint_state_topic_name = joint_state_topic_name.into();
        Arc::new(Lazy::new(Box::new(move || {
            Box::new(JointStateProviderFromJointState::new(
                SubscriberHandler::new(&joint_state_topic_name, 1),
            ))
        })))
    }
}

impl JointStateProvider for RosControlActionClient {
    fn get_joint_state(&self) -> Result<(Vec<String>, Vec<f64>), arci::Error> {
        self.0.joint_state_provider.get_joint_state()
    }
}

impl JointTrajectoryClient for RosControlActionClient {
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
            self,
            &trajectory,
            self.0.send_partial_joints_goal,
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
