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

/// Implement arci::Navigation for ROS2
pub struct Ros2Navigation {
    action_client: r2r::ActionClient<NavigateToPose::Action>,
    /// r2r::Node to handle the action
    node: Arc<Mutex<r2r::Node>>,
    current_goal: Arc<Mutex<Option<r2r::ActionClientGoal<NavigateToPose::Action>>>>,
}

// TODO:
unsafe impl Sync for Ros2Navigation {}

impl Ros2Navigation {
    /// Creates a new `Ros2Navigation` from ROS2 context and name of action.
    #[track_caller]
    pub fn new(ctx: r2r::Context, action_name: &str) -> Self {
        // TODO: Consider using unique name
        let node = r2r::Node::create(ctx, "nav2_node", "arci_ros2").unwrap();
        Self::from_node(node, action_name)
    }

    /// Creates a new `Ros2Navigation` from ROS2 node and name of action.
    #[track_caller]
    pub fn from_node(mut node: r2r::Node, action_name: &str) -> Self {
        let action_client = node
            .create_action_client::<NavigateToPose::Action>(action_name)
            .unwrap();
        Self {
            action_client,
            node: Arc::new(Mutex::new(node)),
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
        let is_done = Arc::new(AtomicBool::new(false));
        let is_done_clone = is_done.clone();
        let current_goal = self.current_goal.clone();
        let action_client = self.action_client.clone();
        let is_available = node.lock().is_available(&self.action_client).unwrap();
        let (sender, receiver) = tokio::sync::oneshot::channel();
        let frame_id = frame_id.to_owned();
        std::thread::spawn(move || {
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(async move {
                    let current_goal_clone = current_goal.clone();
                    tokio::spawn(async move {
                        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                        let now = clock.get_now().unwrap();
                        let goal = NavigateToPose::Goal {
                            pose: msg::PoseStamped {
                                header: Header {
                                    frame_id: frame_id.to_string(),
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
                        tokio::spawn(
                            async move { feedback.for_each(|_| std::future::ready(())).await },
                        );
                        result.await.unwrap(); // TODO: handle goal state
                        is_done.store(true, Ordering::Relaxed);
                    });
                    loop {
                        node.lock().spin_once(std::time::Duration::from_millis(100));
                        tokio::task::yield_now().await;
                        if is_done_clone.load(Ordering::Relaxed) {
                            break;
                        }
                    }
                    *current_goal.lock() = None;
                    // TODO: "canceled" should be an error?
                    let _ = sender.send(());
                });
        });
        let timeout_fut = tokio::time::sleep(timeout);
        let wait = WaitFuture::new(async move {
            tokio::select! {
                _ = receiver => Ok(()),
                _ = timeout_fut => Err(arci::Error::Other(format_err!("timeout {:?}", timeout))),
            }
        });

        Ok(wait)
    }

    fn cancel(&self) -> Result<(), Error> {
        // TODO: current_goal is None until send_goal_request.await is complete.
        //       Therefore, if cancel is called during that period, it will not work correctly.
        if let Some(current_goal) = self.current_goal.lock().take() {
            let fut = current_goal.cancel().map_err(|e| Error::Other(e.into()))?;
            std::thread::spawn(move || {
                tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .unwrap()
                    .block_on(async move {
                        let _ = fut.await;
                    });
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
/// Configuration for Ros2NavigationConfig
pub struct Ros2NavigationConfig {
    /// Name of action nav2_msgs/NavigateToPose
    pub action_name: String,
}
