use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use arci::*;
use r2r::{
    builtin_interfaces::msg::Time, geometry_msgs::msg, nav2_msgs::action::NavigateToPose,
    std_msgs::msg::Header,
};
use serde::{Deserialize, Serialize};
use tracing::{debug, info};

/// Implement arci::Navigation for ROS2
pub struct Ros2Navigation {
    action_client: r2r::ActionClient<NavigateToPose::Action>,
    /// r2r::Node to handle the action
    pub node: Arc<Mutex<r2r::Node>>,
}

// TODO:
unsafe impl Sync for Ros2Navigation {}

impl Ros2Navigation {
    /// Create instance from ROS2 context and name of action
    pub fn new(ctx: r2r::Context, action_name: &str) -> Self {
        // TODO: Use unique name
        let mut node = r2r::Node::create(ctx, "nav2_node", "arci_ros2").unwrap();
        let action_client = node
            .create_action_client::<NavigateToPose::Action>(action_name)
            .unwrap();
        while !node.action_server_available(&action_client).unwrap() {
            std::thread::sleep(std::time::Duration::from_millis(500));
            debug!("waiting action server..");
        }
        debug!("waiting action server..Done");
        Self {
            action_client,
            node: Arc::new(Mutex::new(node)),
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

        let has_reached = Arc::new(Mutex::new(None));

        let cb = Box::new(move |r: NavigateToPose::SendGoal::Response| {
            debug!("got response {:?}", r);
        });

        let feedback_cb = Box::new(move |fb: NavigateToPose::Feedback| {
            debug!("got feedback {:?}", fb);
        });

        let has_reached_set = has_reached.clone();
        let result_cb = Box::new(move |r: NavigateToPose::Result| {
            info!("final result {:?}", r);
            *has_reached_set.lock().unwrap() = Some(r);
        });
        self.action_client
            .send_goal_request(goal, cb, feedback_cb, result_cb)
            .unwrap();
        let spin_node = self.node.clone();
        let start_time = std::time::Instant::now();
        let wait = WaitFuture::new(async move {
            const SLEEP_DURATION: std::time::Duration = std::time::Duration::from_micros(100);
            while has_reached.lock().unwrap().is_none() && start_time.elapsed() < timeout {
                spin_node.lock().unwrap().spin_once(SLEEP_DURATION);
                std::thread::sleep(SLEEP_DURATION);
            }
            // TODO: Handle the result and timeout
            Ok(())
        });

        Ok(wait)
    }

    fn cancel(&self) -> Result<(), Error> {
        todo!();
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
/// Configuration for Ros2NavigationConfig
pub struct Ros2NavigationConfig {
    /// Name of action nav2_msgs/NavigateToPose
    pub action_name: String,
}
