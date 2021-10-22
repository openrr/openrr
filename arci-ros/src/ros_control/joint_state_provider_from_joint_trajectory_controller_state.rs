use msg::control_msgs::JointTrajectoryControllerState;

use crate::{error::Error, msg, JointStateProvider, SubscriberHandler};

pub(crate) struct JointStateProviderFromJointTrajectoryControllerState(
    SubscriberHandler<JointTrajectoryControllerState>,
);

impl JointStateProviderFromJointTrajectoryControllerState {
    pub(crate) fn new(
        subscriber_handler: SubscriberHandler<JointTrajectoryControllerState>,
    ) -> Self {
        subscriber_handler.wait_message(100);
        Self(subscriber_handler)
    }
}

impl JointStateProvider for JointStateProviderFromJointTrajectoryControllerState {
    fn get_joint_state(&self) -> Result<(Vec<String>, Vec<f64>), arci::Error> {
        let state = self
            .0
            .get()?
            .ok_or_else(|| arci::Error::Other(Error::NoJointStateAvailable.into()))?;
        Ok((state.joint_names, state.actual.positions))
    }
}
