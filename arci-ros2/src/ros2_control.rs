use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

use arci::*;
use futures::stream::StreamExt;
use parking_lot::Mutex;
use r2r::{
    builtin_interfaces::{msg as builtin_msg, msg::Time},
    control_msgs::{action::FollowJointTrajectory, msg::JointTrajectoryControllerState},
    std_msgs::msg::Header,
    trajectory_msgs::msg as trajectory_msg,
};
use serde::{Deserialize, Serialize};

use crate::utils;

/// Implement arci::JointTrajectoryClient for ROS2
pub struct Ros2ControlClient {
    state_topic: String,
    action_client: r2r::ActionClient<FollowJointTrajectory::Action>,
    /// r2r::Node to handle the action
    node: Arc<Mutex<r2r::Node>>,
    joint_names: Vec<String>,
}

// TODO:
unsafe impl Sync for Ros2ControlClient {}

impl Ros2ControlClient {
    /// Creates a new `Ros2ControlClient` from ROS2 context and names of action.
    #[track_caller]
    pub fn new(ctx: r2r::Context, action_name: &str) -> Self {
        // TODO: Consider using unique name
        let node = r2r::Node::create(ctx, "ros2_control_node", "arci_ros2").unwrap();
        Self::from_node(node, action_name)
    }

    /// Creates a new `Ros2ControlClient` from ROS2 node and names of action.
    #[track_caller]
    pub fn from_node(mut node: r2r::Node, action_name: &str) -> Self {
        // http://wiki.ros.org/joint_trajectory_controller
        let action_client = node
            .create_action_client::<FollowJointTrajectory::Action>(&format!(
                "{action_name}/follow_joint_trajectory"
            ))
            .unwrap();
        let state_topic = format!("{action_name}/state");
        let node = Arc::new(Mutex::new(node));
        let joints = get_joint_state(node.clone(), &state_topic);
        Self {
            state_topic,
            action_client,
            node,
            joint_names: joints.joint_names,
        }
    }
}

fn get_joint_state(
    node: Arc<Mutex<r2r::Node>>,
    state_topic: &str,
) -> JointTrajectoryControllerState {
    let mut state_subscriber = node
        .lock()
        .subscribe::<JointTrajectoryControllerState>(state_topic)
        .unwrap();
    let is_done = Arc::new(AtomicBool::new(false));
    let is_done_clone = is_done.clone();
    utils::spawn_blocking(async move {
        let handle = tokio::spawn(async move {
            let next = state_subscriber.next().await;
            is_done_clone.store(true, Ordering::SeqCst);
            next
        });
        loop {
            if is_done.load(Ordering::SeqCst) {
                break;
            }
            node.lock().spin_once(Duration::from_millis(100));
            tokio::task::yield_now().await;
        }
        handle.await.unwrap()
    })
    .join()
    .unwrap()
    .unwrap()
}

impl JointTrajectoryClient for Ros2ControlClient {
    fn joint_names(&self) -> Vec<String> {
        self.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        let joints = get_joint_state(self.node.clone(), &self.state_topic);
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
        let node = self.node.clone();
        let is_done = Arc::new(AtomicBool::new(false));
        let is_done_clone = is_done.clone();
        let action_client = self.action_client.clone();
        let is_available = node.lock().is_available(&self.action_client).unwrap();
        let (sender, receiver) = tokio::sync::oneshot::channel();
        let joint_names = self.joint_names.clone();
        utils::spawn_blocking(async move {
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
            loop {
                if is_done_clone.load(Ordering::Relaxed) {
                    break;
                }
                node.lock().spin_once(Duration::from_millis(100));
                tokio::task::yield_now().await;
            }
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

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
/// Configuration for Ros2ControlConfig
pub struct Ros2ControlConfig {
    /// Name of action control_msgs::action::FollowJointTrajectory
    pub action_name: String,
    /// Names of joints.
    #[serde(default)]
    pub joint_names: Vec<String>,
}
