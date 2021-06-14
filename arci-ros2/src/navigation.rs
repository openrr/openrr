use arci::*;
use r2r::geometry_msgs::msg;
use r2r::nav2_msgs::action::NavigateToPose;
use std::sync::{Arc, Mutex};
use std::time::Duration;

struct MyNode(r2r::Node);

pub struct Ros2Navigation {
    action_client: r2r::ActionClient<NavigateToPose::Action>,
    node: Arc<Mutex<MyNode>>,
}

// I don't know this is OK...
unsafe impl Send for Ros2Navigation {}
unsafe impl Sync for Ros2Navigation {}
unsafe impl Send for MyNode {}
unsafe impl Sync for MyNode {}

impl Ros2Navigation {
    pub fn new(ctx: r2r::Context, action_name: &str) -> Self {
        let mut node = r2r::Node::create(ctx, "example_nav2_node", "").unwrap();
        let action_client = node
            .create_action_client::<NavigateToPose::Action>(action_name)
            .unwrap();
        while !node.action_server_available(&action_client).unwrap() {
            std::thread::sleep(std::time::Duration::from_millis(500));
            println!("waiting action server..");
        }
        println!("waiting action server..Done");
        Self {
            action_client,
            node: Arc::new(Mutex::new(MyNode(node))),
        }
    }
}

impl Navigation for Ros2Navigation {
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        _frame_id: &str,
        timeout: Duration,
    ) -> Result<WaitFuture, Error> {
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), goal.rotation.angle());
        let goal = NavigateToPose::Goal {
            pose: msg::PoseStamped {
                pose: msg::Pose {
                    position: msg::Point {
                        x: goal.translation.x,
                        y: goal.translation.y,
                        z: 0.0,
                    },
                    orientation: msg::Quaternion {
                        x: q.coords.x,
                        y: q.coords.y,
                        z: q.coords.z,
                        w: q.coords.w,
                    },
                },
                ..Default::default()
            },
            ..Default::default()
        };

        let has_reached = Arc::new(Mutex::new(None));

        let cb = Box::new(move |r: NavigateToPose::SendGoal::Response| {
            println!("got response {:?}", r);
        });

        let feedback_cb = Box::new(move |fb: NavigateToPose::Feedback| {
            println!("got feedback {:?}", fb);
        });

        let has_reached_set = has_reached.clone();
        let result_cb = Box::new(move |r: NavigateToPose::Result| {
            println!("final result {:?}", r);
            *has_reached_set.lock().unwrap() = Some(r);
        });
        self.action_client
            .send_goal_request(goal, cb, feedback_cb, result_cb)
            .unwrap();
        let spin_node = self.node.clone();
        let wait = WaitFuture::new(async move {
            const SLEEP_DURATION: std::time::Duration = std::time::Duration::from_micros(100);
            while has_reached.lock().unwrap().is_none() {
                spin_node.lock().unwrap().0.spin_once(SLEEP_DURATION);
                std::thread::sleep(SLEEP_DURATION);
            }
            Ok(())
        });

        Ok(wait)
    }

    fn cancel(&self) -> Result<(), Error> {
        todo!();
    }
}
