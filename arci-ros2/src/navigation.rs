use std::{
    pin::pin,
    sync::{Arc, Mutex},
    time::Duration,
};

use anyhow::format_err;
use arci::*;
use futures::stream::StreamExt;
use ros2_client::{builtin_interfaces::Time, *};
use serde::{Deserialize, Serialize};
use tokio::sync::oneshot;
use tracing::debug;

use crate::{
    msg::{geometry_msgs, nav2_msgs::NavigateToPose, std_msgs},
    Node,
};

/// `arci::Navigation` implementation for ROS2.
pub struct Ros2Navigation {
    send_goal_pose: Arc<Mutex<Option<SendGoalPoseRequest>>>,
    cancel: Mutex<Option<oneshot::Sender<()>>>,
    // keep not to be dropped
    _node: Node,
}

struct SendGoalPoseRequest {
    goal: Isometry2<f64>,
    frame_id: String,
    timeout: Duration,
    result_sender: oneshot::Sender<Result<(), Duration>>,
    cancel_receiver: oneshot::Receiver<()>,
}

impl Ros2Navigation {
    /// Creates a new `Ros2Navigation` from nav2_msgs/NavigateToPose action name.
    #[track_caller]
    pub fn new(node: Node, action_name: &str) -> Self {
        let action_client = node
            .create_action_client::<NavigateToPose::Action>(action_name)
            .unwrap();
        let send_goal_pose: Arc<Mutex<Option<SendGoalPoseRequest>>> = Arc::new(Mutex::new(None));
        let recv_goal_pose = send_goal_pose.clone();
        std::thread::spawn(move || {
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(send_goal_pose_thread(action_client, recv_goal_pose));
        });
        Self {
            send_goal_pose,
            cancel: Mutex::new(None),
            _node: node,
        }
    }
}

async fn send_goal_pose_thread(
    action_client: action::ActionClient<NavigateToPose::Action>,
    recv_goal_pose: Arc<Mutex<Option<SendGoalPoseRequest>>>,
) {
    'outer: while Arc::strong_count(&recv_goal_pose) > 1 {
        let req = recv_goal_pose.lock().unwrap().take();
        let Some(SendGoalPoseRequest {
            goal,
            frame_id,
            timeout,
            result_sender,
            mut cancel_receiver,
        }) = req
        else {
            tokio::time::sleep(Duration::from_millis(10)).await;
            continue;
        };

        let goal = NavigateToPose::Goal {
            pose: geometry_msgs::PoseStamped {
                header: std_msgs::Header {
                    frame_id,
                    stamp: Time::now(),
                },
                pose: (&goal).into(),
            },
            ..Default::default()
        };

        let send_goal_req = action_client.async_send_goal(goal);
        let timeout_fut = tokio::time::sleep(timeout);
        let (goal_id, goal_response) = tokio::select! { biased;
            _res = &mut cancel_receiver => {
                // res.is_ok == canceled, res.is_err == new request set
                let _ = result_sender.send(Ok(())); // TODO: "canceled" should be an error?
                continue;
            }
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
                tokio::select! { biased;
                    // res.is_ok == canceled, res.is_err == new request set
                    _res = &mut cancel_receiver => {
                        // TODO: handle result
                        let _res = action_client.async_cancel_goal(goal_id, Time::now()).await;
                        let _ = result_sender.send(Ok(())); // TODO: "canceled" should be an error?
                        continue 'outer;
                    }

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

impl Navigation for Ros2Navigation {
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: Duration,
    ) -> Result<WaitFuture, Error> {
        let (result_sender, result_receiver) = tokio::sync::oneshot::channel();
        let (cancel_sender, cancel_receiver) = tokio::sync::oneshot::channel();
        *self.cancel.lock().unwrap() = Some(cancel_sender);
        *self.send_goal_pose.lock().unwrap() = Some(SendGoalPoseRequest {
            goal,
            frame_id: frame_id.to_owned(),
            timeout,
            result_sender,
            cancel_receiver,
        });
        let timeout_fut = tokio::time::sleep(timeout);
        let wait = WaitFuture::new(async move {
            tokio::select! {
                res = result_receiver => {
                    res
                        .map_err(|e| arci::Error::Other(e.into()))?
                        .map_err(|timeout| arci::Error::Other(format_err!("timeout {timeout:?}")))
                }
                _ = timeout_fut => Err(arci::Error::Other(format_err!("timeout {timeout:?}"))),
            }
        });

        Ok(wait)
    }

    fn cancel(&self) -> Result<(), Error> {
        let cancel_sender = self.cancel.lock().unwrap().take();
        if let Some(cancel_sender) = cancel_sender {
            let _ = cancel_sender.send(());
        }
        Ok(())
    }
}

/// Configuration for `Ros2Navigation`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ros2NavigationConfig {
    /// Action name for nav2_msgs/NavigateToPose.
    pub action_name: String,
}
