use std::sync::Arc;

use arci::JointTrajectoryClient;
use once_cell::sync::Lazy;

use crate::JointTrajectoryClientWrapperConfig;

pub trait JointStateProvider {
    fn get_joint_state(&self) -> Result<(Vec<String>, Vec<f64>), arci::Error>;
}

pub trait RosControlClientBuilder {
    fn build_joint_state_provider(
        &self,
        joint_state_topic_name: impl Into<String>,
    ) -> Arc<LazyJointStateProvider>;
    fn build_joint_trajectory_client(
        &self,
        lazy: bool,
        joint_state_provider: Arc<LazyJointStateProvider>,
    ) -> Result<Arc<dyn JointTrajectoryClient>, arci::Error>;
    fn state_topic(&self) -> String;
    fn wrapper_config(&self) -> &JointTrajectoryClientWrapperConfig;
    fn name(&self) -> &str;
}

pub(crate) type LazyJointStateProvider = Lazy<
    Box<dyn JointStateProvider + Send + Sync>,
    Box<dyn FnOnce() -> Box<dyn JointStateProvider + Send + Sync> + Send + Sync>,
>;
