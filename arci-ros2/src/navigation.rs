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
use r2r::{builtin_interfaces::msg::Time, geometry_msgs::msg, std_msgs::msg::Header, QosProfile};
use serde::{Deserialize, Serialize};
use tokio::sync::oneshot;

use crate::{utils, Node};

/// `arci::Navigation` implementation for ROS2.
pub struct Ros2Navigation {
    goal_pose_publisher: Arc<Mutex<r2r::Publisher<msg::PoseStamped>>>,
    amcl_pose_topic_name: String,
    // keep not to be dropped
    node: Node,
    current_goal: Arc<Mutex<Option<oneshot::Sender<()>>>>,
}

impl Ros2Navigation {
    /// Creates a new `Ros2Navigation` from geometry_msgs/PoseStamped and geometry_msgs/PoseWithCovarianceStamped topic names.
    #[track_caller]
    pub fn new(node: Node, goal_pose_topic_name: &str, amcl_pose_topic_name: &str) -> Self {
        let goal_pose_publisher = Arc::new(Mutex::new(
            node.r2r()
                .create_publisher(goal_pose_topic_name, r2r::QosProfile::default())
                .unwrap(),
        ));
        Self {
            goal_pose_publisher,
            amcl_pose_topic_name: amcl_pose_topic_name.to_owned(),
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
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let mut goal_pose = msg::PoseStamped {
            header: Header {
                frame_id: frame_id.to_owned(),
                stamp: Time { sec: 0, nanosec: 0 },
            },
            pose: to_pose_msg(&goal),
        };

        let (res_sender, res_receiver) = oneshot::channel();
        let (cancel, mut canceled) = oneshot::channel();
        *self.current_goal.lock() = Some(cancel);
        let goal_pose_publisher = self.goal_pose_publisher.clone();
        let mut amcl_pose_subscriber = self
            .node
            .r2r()
            .subscribe::<msg::PoseWithCovarianceStamped>(
                &self.amcl_pose_topic_name,
                QosProfile::default(),
            )
            .unwrap();
        utils::spawn_blocking(async move {
            let mut changed = false;
            let finished = Arc::new(AtomicBool::new(false));
            let finished_clone = finished.clone();
            tokio::spawn(async move {
                while !finished_clone.load(Ordering::Relaxed) {
                    let now = clock.get_now().unwrap();
                    goal_pose.header.stamp = Time {
                        sec: now.as_secs() as i32,
                        nanosec: now.subsec_nanos(),
                    };
                    if let Err(e) = goal_pose_publisher.lock().publish(&goal_pose) {
                        tracing::error!("r2r publish error: {e:?}");
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            });
            while canceled
                .try_recv()
                .map_or_else(|e| e == oneshot::error::TryRecvError::Empty, |()| false)
            {
                let sleep = tokio::time::sleep(timeout);
                tokio::pin!(sleep);
                tokio::select! { biased;
                    Some(_) = amcl_pose_subscriber.next() => {
                        changed = true;
                    },
                    _ = &mut sleep => {
                        // TODO: we have no way to this is time out or finished
                        if changed {
                            let _ = res_sender.send(Ok(()));
                        } else {
                            let _ = res_sender.send(Err(format_err!("timeout {timeout:?}").into()));
                        }
                        finished.store(true, Ordering::Relaxed);
                        return;
                    },
                }
            }
            finished.store(true, Ordering::Relaxed);
            let _ = res_sender.send(Err(arci::Error::Canceled {
                message: String::new(),
            }));
        });

        Ok(WaitFuture::new(async move { res_receiver.await.unwrap() }))
    }

    fn cancel(&self) -> Result<(), Error> {
        *self.current_goal.lock() = None;
        Ok(())
    }
}

/// Configuration for `Ros2Navigation`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ros2NavigationConfig {
    /// Topic name for geometry_msgs/PoseStamped.
    pub goal_pose_topic: String,
    /// Topic name for geometry_msgs/PoseWithCovarianceStamped.
    pub amcl_pose_topic: String,
}
