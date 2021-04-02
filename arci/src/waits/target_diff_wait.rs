use super::CompleteCondition;
use crate::error::Error;
use crate::traits::{JointTrajectoryClient, WaitTrait};

pub struct TargetDiffWait<'a> {
    client: &'a dyn JointTrajectoryClient,
    target_positions: Vec<f64>,
    duration_sec: f64,
    pub condition: &'a dyn CompleteCondition,
}

impl<'a> TargetDiffWait<'a> {
    /// Creates a new `TargetDiffWait`.
    pub fn new(
        client: &'a dyn JointTrajectoryClient,
        target_positions: Vec<f64>,
        duration_sec: f64,
        condition: &'a dyn CompleteCondition,
    ) -> Self {
        Self {
            client,
            target_positions,
            duration_sec,
            condition,
        }
    }

    /// Creates a new boxed `TargetDiffWait`.
    pub fn new_boxed(
        client: &'a dyn JointTrajectoryClient,
        target_positions: Vec<f64>,
        duration_sec: f64,
        condition: &'a dyn CompleteCondition,
    ) -> Box<Self> {
        Box::new(Self::new(client, target_positions, duration_sec, condition))
    }
}

impl WaitTrait for TargetDiffWait<'_> {
    fn wait(&self) -> Result<(), Error> {
        self.condition
            .wait(self.client, &self.target_positions, self.duration_sec)
    }
}
