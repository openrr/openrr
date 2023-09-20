use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

use anyhow::format_err;
use arci::*;
use futures::stream::StreamExt;
use parking_lot::Mutex;
use r2r::{
    builtin_interfaces::msg::Time, geometry_msgs::msg, nav2_msgs::action::NavigateToPose,
    std_msgs::msg::Header,
};
use serde::{Deserialize, Serialize};

use crate::{utils, Node};

/// `arci::Navigation` implementation for ROS2.
pub struct Ros2Navigation {
    action_client: r2r::ActionClient<NavigateToPose::Action>,
    /// r2r::Node to handle the action
    node: Node,
    current_goal: Arc<Mutex<Option<r2r::ActionClientGoal<NavigateToPose::Action>>>>,
}

impl Ros2Navigation {
    /// Creates a new `Ros2Navigation` from nav2_msgs/NavigateToPose action name.
    #[track_caller]
    pub fn new(node: Node, action_name: &str) -> Self {
        let action_client = node
            .r2r()
            .create_action_client::<NavigateToPose::Action>(action_name)
            .unwrap();
        Self {
            action_client,
            node,
            current_goal: Arc::new(Mutex::new(None)),
        }
    }
}

fn to_pose_msg(pose: &Isometry2<f64>) -> msg::Pose {
    let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), pose.rotation.angle());
    msg::Pose {
        position: msg::Point {
            x: pose.translation.x,
            y: pose.translation.y,
            z: 0.0,
        },
        orientation: msg::Quaternion {
            x: q.coords.x,
            y: q.coords.y,
            z: q.coords.z,
            w: q.coords.w,
        },
    }
}

impl Navigation for Ros2Navigation {
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: Duration,
    ) -> Result<WaitFuture, Error> {
        let node = self.node.clone();
        let current_goal = self.current_goal.clone();
        let action_client = self.action_client.clone();
        let is_available = node.r2r().is_available(&self.action_client).unwrap();
        let (sender, receiver) = tokio::sync::oneshot::channel();
        let frame_id = frame_id.to_owned();
        utils::spawn_blocking(async move {
            let is_done = Arc::new(AtomicBool::new(false));
            let is_done_clone = is_done.clone();
            let current_goal_clone = current_goal.clone();
            tokio::spawn(async move {
                let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                let now = clock.get_now().unwrap();
                let goal = NavigateToPose::Goal {
                    pose: msg::PoseStamped {
                        header: Header {
                            frame_id,
                            stamp: Time {
                                sec: now.as_secs() as i32,
                                nanosec: now.subsec_nanos(),
                            },
                        },
                        pose: to_pose_msg(&goal),
                    },
                    ..Default::default()
                };

                is_available.await.unwrap();
                let send_goal_request = action_client.send_goal_request(goal).unwrap();
                let (goal, result, feedback) = send_goal_request.await.unwrap();
                *current_goal_clone.lock() = Some(goal.clone());
                tokio::spawn(async move { feedback.for_each(|_| std::future::ready(())).await });
                result.await.unwrap(); // TODO: handle goal state
                is_done.store(true, Ordering::Relaxed);
            });
            utils::wait(is_done_clone).await;
            *current_goal.lock() = None;
            // TODO: "canceled" should be an error?
            let _ = sender.send(());
        });
        let timeout_fut = tokio::time::sleep(timeout);
        let wait = WaitFuture::new(async move {
            tokio::select! {
                _ = receiver => Ok(()),
                _ = timeout_fut => Err(arci::Error::Other(format_err!("timeout {timeout:?}"))),
            }
        });

        Ok(wait)
    }

    fn cancel(&self) -> Result<(), Error> {
        // TODO: current_goal is None until send_goal_request.await is complete.
        //       Therefore, if cancel is called during that period, it will not work correctly.
        if let Some(current_goal) = self.current_goal.lock().take() {
            let fut = current_goal.cancel().map_err(|e| Error::Other(e.into()))?;
            utils::spawn_blocking(async move {
                let _ = fut.await;
            });
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
