use crate::{JointStateProvider, SubscriberHandler, error::Error, msg::sensor_msgs::JointState};

pub(crate) struct JointStateProviderFromJointState(SubscriberHandler<JointState>);

impl JointStateProviderFromJointState {
    pub(crate) fn new(subscriber_handler: SubscriberHandler<JointState>) -> Self {
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
