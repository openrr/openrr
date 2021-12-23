#![cfg(feature = "ros2")]

use std::{
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc,
    },
    time::Duration,
};

use arci::*;
use arci_ros2::{r2r, Ros2ControlClient};
use futures::{
    future::{self, Either},
    stream::{Stream, StreamExt},
};
use parking_lot::Mutex;
use r2r::control_msgs::action::FollowJointTrajectory;

// (node_name, action_name)
fn node_and_action_name() -> (String, String) {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    let n = COUNT.fetch_add(1, Ordering::SeqCst);
    let node_name = format!("test_ros2_control_node_{}", n);
    let action_name = format!("/test_ros2_control_{}", n);
    (node_name, action_name)
}

#[tokio::test]
async fn test_control() {
    let (node_name, action_name) = &node_and_action_name();
    let ctx = r2r::Context::create().unwrap();
    let client = Ros2ControlClient::new(ctx.clone(), action_name, vec!["j1".into(), "j2".into()]);
    let node = Arc::new(Mutex::new(
        r2r::Node::create(ctx, node_name, "arci_ros2").unwrap(),
    ));

    let server_requests = node
        .lock()
        .create_action_server::<FollowJointTrajectory::Action>(action_name)
        .unwrap();

    let node_cb = node.clone();
    tokio::spawn(test_control_server(node_cb, server_requests));

    std::thread::spawn(move || loop {
        node.lock().spin_once(Duration::from_millis(100));
    });

    client
        .send_joint_positions(vec![1.0, 1.0], Duration::from_secs(80))
        .unwrap()
        .await
        .unwrap();
}

async fn run_goal(
    node: Arc<Mutex<r2r::Node>>,
    g: r2r::ActionServerGoal<FollowJointTrajectory::Action>,
) -> FollowJointTrajectory::Result {
    let mut timer = node // local timer, will be dropped after this request is processed.
        .lock()
        .create_wall_timer(Duration::from_millis(800))
        .expect("could not create timer");

    let mut feedback_msg = FollowJointTrajectory::Feedback::default();
    g.publish_feedback(feedback_msg.clone()).expect("fail");

    feedback_msg.actual = feedback_msg.desired.clone();
    g.publish_feedback(feedback_msg.clone()).expect("fail");
    println!("Sending feedback: {:?}", feedback_msg);
    timer.tick().await.unwrap();

    FollowJointTrajectory::Result::default()
}

async fn test_control_server(
    node: Arc<Mutex<r2r::Node>>,
    mut requests: impl Stream<Item = r2r::ActionServerGoalRequest<FollowJointTrajectory::Action>>
        + Unpin,
) {
    while let Some(req) = requests.next().await {
        println!(
            "Got goal request with trajectory {:?}, goal id: {}",
            req.goal.trajectory, req.uuid
        );
        let (mut g, mut cancel) = req.accept().expect("could not accept goal");

        let goal_fut = tokio::task::spawn(run_goal(node.clone(), g.clone()));

        match future::select(goal_fut, cancel.next()).await {
            Either::Left((result, _)) => {
                let result = result.unwrap();
                g.succeed(result).expect("could not send result");
            }
            Either::Right((request, _)) => {
                if let Some(request) = request {
                    panic!("got cancel request: {}", request.uuid);
                }
            }
        }
    }
}
