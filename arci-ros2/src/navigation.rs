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
    action_client: Arc<Mutex<r2r::ActionClient<NavigateToPose::Action>>>,
    /// r2r::Node to handle the action
    node: Arc<Mutex<r2r::Node>>,
    is_plugin: bool,
}

// TODO:
//unsafe impl Sync for Ros2Navigation {}

impl Ros2Navigation {
    /// Create instance from ROS2 context and name of action
    pub fn new(ctx: r2r::Context, action_name: &str) -> Self {
        Self::new_inner(ctx, action_name, false)
    }

    /// Create instance from ROS2 context and name of action for plugin
    pub fn new_for_plugin(ctx: r2r::Context, action_name: &str) -> Self {
        Self::new_inner(ctx, action_name, true)
    }

    fn new_inner(ctx: r2r::Context, action_name: &str, is_plugin: bool) -> Self {
        // TODO: Use unique name
        let mut node = r2r::Node::create(
            ctx,
            &format!("arci_ros2_nav2_node{}", rand::random::<u32>()),
            "",
        )
        .expect("failed to create navigation node");
        println!("aaa");
        let action_client = Arc::new(Mutex::new(
            node.create_action_client::<NavigateToPose::Action>(action_name)
                .unwrap(),
        ));
        let arc_node = Arc::new(Mutex::new(node));
        let spin_node = arc_node.clone();
        std::thread::spawn(move || {
            const SLEEP_DURATION: Duration = Duration::from_millis(100);
            spin_node.lock().unwrap().spin_once(SLEEP_DURATION);
            std::thread::sleep(SLEEP_DURATION);
        });
        let is_available_node = arc_node.clone();
        let is_available_client = action_client.clone();
        // TODO: It seems that `is_available` is not working (It blocks forever)
        /*
        block_in_place(
            is_plugin,
            is_available_node
                .lock()
                .unwrap()
                .is_available(&*is_available_client.lock().unwrap())
                .expect("failed to wait navigation action"),
        )
        .unwrap();
        */
        //.expect("failed to find action");
        println!("bbb");
        debug!("waiting action server..Done");
        Self {
            action_client,
            node: arc_node,
            is_plugin,
        }
    }
}

fn block_in_place<T>(is_plugin: bool, f: impl std::future::Future<Output = T>) -> T {
    if is_plugin {
        tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(f)
    } else {
        tokio::task::block_in_place(|| tokio::runtime::Handle::current().block_on(f))
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
        let has_reached_set = has_reached.clone();
        let action_client = self.action_client.clone();

        std::thread::sleep(std::time::Duration::from_micros(1000));
        let (_goal, result, _feedback) = block_in_place(self.is_plugin, async move {
            println!("ccc");
            action_client
                .lock()
                .unwrap()
                .send_goal_request(goal)
                .expect("failed to send goal")
                .await
                .expect("goal rejected by server")
        });

        let wait = WaitFuture::new(async move {
            println!("ddd");
            match result.await {
                Ok((_status, r)) => {
                    info!("final result {:?}", r);
                    *has_reached_set.lock().unwrap() = Some(r);
                }
                Err(e) => println!("failed {:?}", e),
            };
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
