use std::sync::Arc;

use arci::{EachJointDiffCondition, JointTrajectoryClient, SetCompleteCondition};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::{
    JointTrajectoryClientWrapperConfig, LazyJointStateProvider, RosControlActionClient,
    RosControlClientBuilder,
};

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
