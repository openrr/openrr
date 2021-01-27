use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("arci_ros: No joint_state is available")]
    NoJointStateAvailable,
    #[error("arci_ros: length mismatch (model = {}, input = {})", model, input)]
    LengthMismatch { model: usize, input: usize },
    #[error("arci_ros: ActionResultTimeout")]
    ActionResultTimeout,
    #[error("arci_ros: ActionResultNotSuccess {}", .0)]
    ActionResultNotSuccess(String),
    #[error("arci_ros: ActionResultPreempted {}", .0)]
    ActionResultPreempted(String),
    #[error("arci_ros: ActionGoalSendingFailure")]
    ActionGoalSendingFailure,
    #[error("arci_ros: ActionCancelSendingFailure")]
    ActionCancelSendingFailure,
    #[error("arci_ros: arci: {:?}", .0)]
    Arci(#[from] arci::Error),
    #[error("arci_ros: Other: {:?}", .0)]
    Other(#[from] anyhow::Error),
}
