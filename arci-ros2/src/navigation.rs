use arci::*;
use r2r::{
    builtin_interfaces::msg::Time, geometry_msgs::msg, nav2_msgs::action::NavigateToPose,
    std_msgs::msg::Header,
};
use serde::{Deserialize, Serialize};
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};
use tracing::debug;

/// Implement arci::Navigation for ROS2
pub struct Ros2Navigation {
    //action_client: Arc<Mutex<r2r::ActionClient<NavigateToPose::Action>>>,
    /// r2r::Node to handle the action
    node: Arc<Mutex<r2r::Node>>,
    action_name: String,
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
        let node = Arc::new(Mutex::new(
            r2r::Node::create(
                ctx,
                &format!("arci_ros2_nav2_node{}", rand::random::<u32>()),
                "",
            )
            .expect("failed to create navigation node"),
        ));
        println!("aaab");
        Self {
            //action_client,
            node,
            action_name: action_name.to_owned(),
            is_plugin,
            //handle,
        }
    }

    /// Send
    pub fn send_goal_pose2(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: Duration,
    ) -> Result<WaitFuture, Error> {
        let node = self.node.clone();
        let has_reached = Arc::new(Mutex::new(false));
        let has_reached_set = has_reached.clone();
        let client = self
            .node
            .lock()
            .unwrap()
            .create_action_client::<NavigateToPose::Action>(&self.action_name)
            .unwrap();
        let frame_id = frame_id.to_string();
        let spin_node = node.clone();
        let spin_handle = spawn_blocking(self.is_plugin, move || loop {
            spin_node
                .lock()
                .unwrap()
                .spin_once(std::time::Duration::from_millis(100));
        });
        println!("abcde");
        //let task_handle = tokio::task::spawn(async move {
        block_in_place(self.is_plugin, async move {
            println!("111");
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
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
            println!("before wait");
            // TODO: It seems that `is_available` is not working (It blocks forever)
            /*
            node.lock()
                .unwrap()
                .is_available(&client)
                .expect("failed to wait navigation action")
                .await
                .unwrap();
            //.expect("failed to find action");
            */
            println!("bbb");
            debug!("waiting action server..Done");

            //    let action_client = self.action_client.clone();
            println!("444");
            let (_goal, result, _feedback) = client
                .send_goal_request(goal)
                .expect("failed to send goal")
                .await
                .expect("goal rejected by server");
            println!("555");
            match result.await {
                Ok((status, msg)) => {
                    println!("got result {} with msg {:?}", status, msg);
                    *has_reached_set.lock().unwrap() = true;
                }
                Err(e) => println!("action failed: {:?}", e),
            }
        });

        let wait = WaitFuture::new(async move {
            println!("ddd");
            for _i in 0..100 {
                println!("waiting");
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                if *has_reached.lock().unwrap() {
                    return Ok(());
                }
            }
            spin_handle.await.unwrap();
            Ok(())
        });
        println!("fffff");
        Ok(wait)
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

fn spawn_blocking(
    is_plugin: bool,
    f: impl FnOnce() -> () + Send + 'static,
) -> tokio::task::JoinHandle<()> {
    if is_plugin {
        tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .unwrap()
            .spawn_blocking(f)
    } else {
        tokio::task::spawn_blocking(f)
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
        //Ok(WaitFuture::ready())
        self.send_goal_pose2(goal, frame_id, timeout)
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
