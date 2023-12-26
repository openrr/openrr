use std::{
    pin::pin,
    sync::{Arc, Mutex, RwLock},
    time::Duration,
};

use anyhow::format_err;
use arci::*;
use futures::stream::StreamExt;
use ros2_client::{
    builtin_interfaces::{self, Time},
    *,
};
use serde::{Deserialize, Serialize};
use tokio::sync::oneshot;
use tracing::debug;

use crate::{
    msg::{
        control_msgs::{FollowJointTrajectory, JointTrajectoryControllerState},
        std_msgs::Header,
        trajectory_msgs,
    },
    utils, Node,
};

/// `arci::JointTrajectoryClient` implementation for ROS2.
pub struct Ros2ControlClient {
    send_joint_trajectory: Arc<Mutex<Option<SendJointTrajectoryRequest>>>,
    joint_names: Vec<String>,
    joint_state: Arc<RwLock<JointTrajectoryControllerState>>,
    // keep not to be dropped
    _node: Node,
}

struct SendJointTrajectoryRequest {
    trajectory: Vec<TrajectoryPoint>,
    result_sender: oneshot::Sender<Result<(), Duration>>,
}

impl Ros2ControlClient {
    /// Creates a new `Ros2ControlClient` from control_msgs/FollowJointTrajectory action name.
    #[track_caller]
    pub fn new(node: Node, action_name: &str) -> Result<Self, Error> {
        // http://wiki.ros.org/joint_trajectory_controller
        let action_client = node.create_action_client::<FollowJointTrajectory::Action>(
            &format!("{action_name}/follow_joint_trajectory"),
        )?;

        let state_topic =
            node.create_topic::<JointTrajectoryControllerState>(&format!("{action_name}/state"))?;
        let mut state_subscriber =
            node.create_subscription::<JointTrajectoryControllerState>(&state_topic)?;
        let Some(joint_state) = utils::subscribe_one(&mut state_subscriber, Duration::from_secs(1))
        else {
            return Err(Error::Connection {
                message: format!(
                    "Failed to get joint_state from {}",
                    rustdds::TopicDescription::name(&state_topic)
                ),
            });
        };
        let joint_names = joint_state.joint_names.clone();
        let joint_state = Arc::new(RwLock::new(joint_state));
        utils::subscribe_thread(state_subscriber, joint_state.clone(), |state| state);
        let send_joint_trajectory: Arc<Mutex<Option<SendJointTrajectoryRequest>>> =
            Arc::new(Mutex::new(None));
        let recv_joint_trajectory = send_joint_trajectory.clone();
        let joint_names_clone = joint_names.clone();
        std::thread::spawn(move || {
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(send_joint_trajectory_thread(
                    action_client,
                    recv_joint_trajectory,
                    joint_names_clone,
                ));
        });

        Ok(Self {
            send_joint_trajectory,
            joint_names,
            joint_state,
            _node: node,
        })
    }
}

async fn send_joint_trajectory_thread(
    action_client: action::ActionClient<FollowJointTrajectory::Action>,
    recv_joint_trajectory: Arc<Mutex<Option<SendJointTrajectoryRequest>>>,
    joint_names: Vec<String>,
) {
    while Arc::strong_count(&recv_joint_trajectory) > 1 {
        let req = recv_joint_trajectory.lock().unwrap().take();
        let Some(SendJointTrajectoryRequest {
            trajectory,
            result_sender,
        }) = req
        else {
            tokio::time::sleep(Duration::from_millis(10)).await;
            continue;
        };

        let goal = FollowJointTrajectory::Goal {
            trajectory: trajectory_msgs::JointTrajectory {
                joint_names: joint_names.clone(),
                points: trajectory
                    .into_iter()
                    .map(|tp| trajectory_msgs::JointTrajectoryPoint {
                        velocities: tp
                            .velocities
                            .unwrap_or_else(|| vec![0.0; tp.positions.len()]),
                        positions: tp.positions,
                        time_from_start: builtin_interfaces::Duration {
                            sec: tp.time_from_start.as_secs().try_into().unwrap_or(i32::MAX),
                            nanosec: tp.time_from_start.subsec_nanos(),
                        },
                        ..Default::default()
                    })
                    .collect(),
                header: Header {
                    stamp: Time::now(),
                    ..Default::default()
                },
            },
            ..Default::default()
        };

        let send_goal_req = action_client.async_send_goal(goal);
        let timeout = Duration::from_secs(10); // TODO
        let timeout_fut = tokio::time::sleep(timeout);
        let (goal_id, goal_response) = tokio::select! {
            res = send_goal_req => res.unwrap(),
            _ = timeout_fut => {
                let _ = result_sender.send(Err(timeout));
                continue;
            }
        };
        if goal_response.accepted {
            let mut feedback_stream = pin!(action_client.feedback_stream(goal_id));
            let mut status_stream = pin!(action_client.all_statuses_stream());
            let mut result_fut = pin!(action_client.async_request_result(goal_id));

            loop {
                tokio::select! {
                    action_result = &mut result_fut => {
                        match action_result {
                            Ok((goal_status, result)) => {
                                debug!("<<< Action Result: {result:?} Status: {goal_status:?}");
                            }
                            Err(e) => debug!("<<< Action Result error {e:?}"),
                        }
                        break;
                    }

                    Some(feedback) = feedback_stream.next() => debug!("<<< Feedback: {feedback:?}"),

                    Some(status) = status_stream.next() => {
                        match status {
                            Ok(status) => match status.status_list.iter().find(|gs| gs.goal_info.goal_id == goal_id) {
                                Some(action_msgs::GoalStatus{goal_info:_, status}) => debug!("<<< Status: {status:?}"),
                                None => debug!("<<< Status: Our status is missing: {:?}", status.status_list),
                            },
                            Err(e) => debug!("<<< Status: {e:?}"),
                        }
                    }
                }
            }
        }
        let _ = result_sender.send(Ok(()));
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
        let (result_sender, result_receiver) = tokio::sync::oneshot::channel();
        *self.send_joint_trajectory.lock().unwrap() = Some(SendJointTrajectoryRequest {
            trajectory,
            result_sender,
        });
        // let node = self.node.clone();
        // let action_client = self.action_client.clone();
        // let is_available = node.r2r().is_available(&self.action_client).unwrap();
        // let (sender, receiver) = tokio::sync::oneshot::channel();
        // let joint_names = self.joint_names.clone();
        // tokio::spawn(async move {
        //     let is_done = Arc::new(AtomicBool::new(false));
        //     let is_done_clone = is_done.clone();
        //     tokio::spawn(async move {
        //         let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        //         let now = clock.get_now().unwrap();
        //         let goal = FollowJointTrajectory::Goal {
        //             trajectory: trajectory_msg::JointTrajectory {
        //                 joint_names,
        //                 points: trajectory
        //                     .into_iter()
        //                     .map(|tp| trajectory_msg::JointTrajectoryPoint {
        //                         velocities: tp
        //                             .velocities
        //                             .unwrap_or_else(|| vec![0.0; tp.positions.len()]),
        //                         positions: tp.positions,
        //                         time_from_start: builtin_interfaces::Duration {
        //                             sec: tp
        //                                 .time_from_start
        //                                 .as_secs()
        //                                 .try_into()
        //                                 .unwrap_or(i32::MAX),
        //                             nanosec: tp.time_from_start.subsec_nanos(),
        //                         },
        //                         ..Default::default()
        //                     })
        //                     .collect(),
        //                 header: Header {
        //                     stamp: Time {
        //                         sec: now.as_secs() as i32,
        //                         nanosec: now.subsec_nanos(),
        //                     },
        //                     ..Default::default()
        //                 },
        //             },
        //             ..Default::default()
        //         };
        //         is_available.await.unwrap();
        //         let send_goal_request = action_client.send_goal_request(goal).unwrap();
        //         let (_goal, result, feedback) = send_goal_request.await.unwrap();
        //         tokio::spawn(async move { feedback.for_each(|_| std::future::ready(())).await });
        //         result.await.unwrap(); // TODO: handle goal state
        //         is_done.store(true, Ordering::Relaxed);
        //     });
        //     utils::wait(is_done_clone).await;
        //     // TODO: "canceled" should be an error?
        //     let _ = sender.send(());
        // });
        let wait = WaitFuture::new(async move {
            result_receiver
                .await
                .map_err(|e| arci::Error::Other(e.into()))?
                .map_err(|timeout| arci::Error::Other(format_err!("timeout {timeout:?}")))
        });
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
