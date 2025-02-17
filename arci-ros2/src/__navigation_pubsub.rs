use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::Duration,
};

use anyhow::format_err;
use arci::*;
// use futures::stream::StreamExt;
use ros2_client::builtin_interfaces::Time;
use serde::{Deserialize, Serialize};
use tokio::sync::oneshot;

use crate::{
    msg::{geometry_msgs, std_msgs},
    Node,
};

/// `arci::Navigation` implementation for ROS2.
pub struct Ros2Navigation {
    goal_pose_publisher: Arc<Mutex<ros2_client::Publisher<geometry_msgs::PoseStamped>>>,
    amcl_pose_topic_name: String,
    // keep not to be dropped
    _node: Node,
    current_goal: Arc<Mutex<Option<oneshot::Sender<()>>>>,
}

impl Ros2Navigation {
    /// Creates a new `Ros2Navigation` from geometry_msgs/PoseStamped and geometry_msgs/PoseWithCovarianceStamped topic names.
    #[track_caller]
    pub fn new(node: Node, goal_pose_topic_name: &str, amcl_pose_topic_name: &str) -> Self {
        let goal_pose_topic = node
            .create_topic::<geometry_msgs::PoseStamped>(goal_pose_topic_name)
            .unwrap();
        let goal_pose_publisher =
            Arc::new(Mutex::new(node.create_publisher(&goal_pose_topic).unwrap()));
        Self {
            goal_pose_publisher,
            amcl_pose_topic_name: amcl_pose_topic_name.to_owned(),
            _node: node,
            current_goal: Arc::new(Mutex::new(None)),
        }
    }
}

impl Navigation for Ros2Navigation {
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: Duration,
    ) -> Result<WaitFuture, Error> {
        let mut goal_pose = geometry_msgs::PoseStamped {
            header: std_msgs::Header {
                frame_id: frame_id.to_owned(),
                stamp: Time::now(),
            },
            pose: (&goal).into(),
        };

        let (res_sender, res_receiver) = oneshot::channel();
        let (cancel, mut canceled) = oneshot::channel();
        *self.current_goal.lock().unwrap() = Some(cancel);
        let goal_pose_publisher = self.goal_pose_publisher.clone();
        // let mut amcl_pose_subscriber = self
        //     .node
        //     .lock()
        //     .unwrap()
        //     .subscribe::<msg::PoseWithCovarianceStamped>(
        //         &self.amcl_pose_topic_name,
        //         QosProfile::default(),
        //     )
        //     .unwrap();
        tokio::spawn(async move {
            let mut changed = false;
            let finished = Arc::new(AtomicBool::new(false));
            let finished_clone = finished.clone();
            tokio::spawn(async move {
                while !finished_clone.load(Ordering::Relaxed) {
                    goal_pose.header.stamp = Time::now();
                    if let Err(e) = goal_pose_publisher
                        .lock()
                        .unwrap()
                        .publish(goal_pose.clone())
                    {
                        tracing::error!("ros2-client publish error: {e:?}");
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
                    // Some(_) = amcl_pose_subscriber.next() => {
                    //     changed = true;
                    // },
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
        *self.current_goal.lock().unwrap() = None;
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
