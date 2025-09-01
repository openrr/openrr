use std::{
    sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};

use arci::*;
use futures::stream::StreamExt;
use r2r::{
    builtin_interfaces::{msg as builtin_msg, msg::Time},
    control_msgs::{action::FollowJointTrajectory, msg::JointTrajectoryControllerState},
    std_msgs::msg::Header,
    trajectory_msgs::msg as trajectory_msg,
};
use serde::{Deserialize, Serialize};

use crate::{Node, utils};

/// `arci::JointTrajectoryClient` implementation for ROS2.
pub struct Ros2ControlClient {
    action_client: r2r::ActionClient<FollowJointTrajectory::Action>,
    // keep not to be dropped
    _node: Node,
    joint_names: Vec<String>,
    joint_state: Arc<RwLock<JointTrajectoryControllerState>>,
}

impl Ros2ControlClient {
    /// Creates a new `Ros2ControlClient` from control_msgs/FollowJointTrajectory action name.
    #[track_caller]
    pub fn new(node: Node, action_name: &str) -> Result<Self, Error> {
        // http://wiki.ros.org/joint_trajectory_controller
        let action_client = node
            .r2r()
            .create_action_client::<FollowJointTrajectory::Action>(&format!(
                "{action_name}/follow_joint_trajectory"
            ))
            .map_err(anyhow::Error::from)?;

        let state_topic = format!("{action_name}/state");
        let mut state_subscriber = node
            .r2r()
            .subscribe::<JointTrajectoryControllerState>(&state_topic, r2r::QosProfile::default())
            .map_err(anyhow::Error::from)?;
        let Some(joint_state) = utils::subscribe_one(&mut state_subscriber, Duration::from_secs(1))
        else {
            return Err(Error::Connection {
                message: format!("Failed to get joint_state from {state_topic}"),
            });
        };
        let joint_names = joint_state.joint_names.clone();
        let joint_state = Arc::new(RwLock::new(joint_state));
        utils::subscribe_thread(state_subscriber, joint_state.clone(), |state| state);

        Ok(Self {
            action_client,
            _node: node,
            joint_names,
            joint_state,
        })
    }
}

impl JointTrajectoryClient for Ros2ControlClient {
    fn joint_names(&self) -> Vec<String> {
        self.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        let joints = self.joint_state.read().unwrap();
        Ok(self
            .joint_names
            .iter()
            .map(|name| {
                joints.actual.positions[joints.joint_names.iter().position(|n| n == name).unwrap()]
            })
            .collect())
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture, arci::Error> {
        self.send_joint_trajectory(vec![TrajectoryPoint {
            positions,
            velocities: None,
            time_from_start: duration,
        }])
    }

    fn send_joint_trajectory(
        &self,
        trajectory: Vec<TrajectoryPoint>,
    ) -> Result<WaitFuture, arci::Error> {
        let action_client = self.action_client.clone();
        let is_available = r2r::Node::is_available(&self.action_client).unwrap();
        let (sender, receiver) = tokio::sync::oneshot::channel();
        let joint_names = self.joint_names.clone();
        tokio::spawn(async move {
            let is_done = Arc::new(AtomicBool::new(false));
            let is_done_clone = is_done.clone();
            tokio::spawn(async move {
                let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                let now = clock.get_now().unwrap();
                let goal = FollowJointTrajectory::Goal {
                    trajectory: trajectory_msg::JointTrajectory {
                        joint_names,
                        points: trajectory
                            .into_iter()
                            .map(|tp| trajectory_msg::JointTrajectoryPoint {
                                velocities: tp
                                    .velocities
                                    .unwrap_or_else(|| vec![0.0; tp.positions.len()]),
                                positions: tp.positions,
                                time_from_start: builtin_msg::Duration {
                                    sec: tp
                                        .time_from_start
                                        .as_secs()
                                        .try_into()
                                        .unwrap_or(i32::MAX),
                                    nanosec: tp.time_from_start.subsec_nanos(),
                                },
                                ..Default::default()
                            })
                            .collect(),
                        header: Header {
                            stamp: Time {
                                sec: now.as_secs() as i32,
                                nanosec: now.subsec_nanos(),
                            },
                            ..Default::default()
                        },
                    },
                    ..Default::default()
                };
                is_available.await.unwrap();
                let send_goal_request = action_client.send_goal_request(goal).unwrap();
                let (_goal, result, feedback) = send_goal_request.await.unwrap();
                tokio::spawn(async move { feedback.for_each(|_| std::future::ready(())).await });
                result.await.unwrap(); // TODO: handle goal state
                is_done.store(true, Ordering::Relaxed);
            });
            utils::wait(is_done_clone).await;
            // TODO: "canceled" should be an error?
            let _ = sender.send(());
        });
        let wait =
            WaitFuture::new(
                async move { receiver.await.map_err(|e| arci::Error::Other(e.into())) },
            );
        Ok(wait)
    }
}

/// Configuration for `Ros2ControlClient`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ros2ControlConfig {
    /// Action name for control_msgs/FollowJointTrajectory.
    pub action_name: String,
    /// Names of joints.
    #[serde(default)]
    pub joint_names: Vec<String>,
}
