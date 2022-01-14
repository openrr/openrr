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
use r2r::{
    control_msgs::{action::FollowJointTrajectory, msg::JointTrajectoryControllerState},
    trajectory_msgs::msg as trajectory_msg,
};

// (node_name, action_name)
fn node_and_action_name() -> (String, String) {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    let n = COUNT.fetch_add(1, Ordering::SeqCst);
    let node_name = format!("test_ros2_control_node_{n}");
    let action_name = format!("/test_ros2_control_{n}");
    (node_name, action_name)
}

#[tokio::test(flavor = "multi_thread")]
async fn test_control() {
    let (node_name, action_name) = &node_and_action_name();
    let ctx = r2r::Context::create().unwrap();

    let node = Arc::new(Mutex::new(
        r2r::Node::create(ctx.clone(), node_name, "arci_ros2").unwrap(),
    ));
    let server_requests = node
        .lock()
        .create_action_server::<FollowJointTrajectory::Action>(&format!(
            "{action_name}/follow_joint_trajectory"
        ))
        .unwrap();
    let publisher = node
        .lock()
        .create_publisher::<JointTrajectoryControllerState>(&format!("{action_name}/state"))
        .unwrap();
    let state = Arc::new(Mutex::new(JointTrajectoryControllerState {
        joint_names: vec!["j1".to_owned(), "j2".to_owned()],
        actual: trajectory_msg::JointTrajectoryPoint {
            positions: vec![0.0; 2],
            ..Default::default()
        },
        ..Default::default()
    }));
    let node_cb = node.clone();
    tokio::spawn(test_control_server(node_cb, server_requests, state.clone()));
    tokio::spawn(async move {
        loop {
            publisher.publish(&state.lock()).unwrap();
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });
    std::thread::spawn(move || loop {
        node.lock().spin_once(Duration::from_millis(100));
    });

    let client = Ros2ControlClient::new(ctx, action_name);
    assert_eq!(client.joint_names(), vec!["j1".to_owned(), "j2".to_owned()]);
    assert_eq!(client.current_joint_positions().unwrap(), vec![0.0, 0.0]);
    client
        .send_joint_positions(vec![1.0, 0.5], Duration::from_secs(80))
        .unwrap()
        .await
        .unwrap();
    assert_eq!(client.current_joint_positions().unwrap(), vec![1.0, 0.5]);
}

async fn run_goal(
    node: Arc<Mutex<r2r::Node>>,
    goal: r2r::ActionServerGoal<FollowJointTrajectory::Action>,
    state: Arc<Mutex<JointTrajectoryControllerState>>,
) -> FollowJointTrajectory::Result {
    let mut timer = node // local timer, will be dropped after this request is processed.
        .lock()
        .create_wall_timer(Duration::from_millis(800))
        .expect("could not create timer");

    state.lock().actual.positions = goal
        .goal
        .trajectory
        .points
        .last()
        .unwrap()
        .positions
        .clone();

    timer.tick().await.unwrap();

    FollowJointTrajectory::Result::default()
}

async fn test_control_server(
    node: Arc<Mutex<r2r::Node>>,
    mut requests: impl Stream<Item = r2r::ActionServerGoalRequest<FollowJointTrajectory::Action>>
        + Unpin,
    state: Arc<Mutex<JointTrajectoryControllerState>>,
) {
    while let Some(req) = requests.next().await {
        println!(
            "Got goal request with trajectory {:?}, goal id: {}",
            req.goal.trajectory, req.uuid
        );
        let (mut goal, mut cancel) = req.accept().expect("could not accept goal");

        let goal_fut = tokio::spawn(run_goal(node.clone(), goal.clone(), state.clone()));

        match future::select(goal_fut, cancel.next()).await {
            Either::Left((result, _)) => {
                let result = result.unwrap();
                goal.succeed(result).expect("could not send result");
            }
            Either::Right((request, _)) => {
                if let Some(request) = request {
                    panic!("got cancel request: {}", request.uuid);
                }
            }
        }
    }
}
