#![cfg(feature = "ros2")]

mod shared;

use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicUsize, Ordering},
    },
    time::Duration,
};

use arci::*;
use arci_ros2::{Node, Ros2ControlClient, r2r};
use futures::{
    future::{self, Either},
    stream::{Stream, StreamExt},
};
use r2r::{
    control_msgs::{action::FollowJointTrajectory, msg::JointTrajectoryControllerState},
    trajectory_msgs::msg as trajectory_msg,
};
use shared::*;

fn action_name() -> String {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    let n = COUNT.fetch_add(1, Ordering::Relaxed);
    format!("/test_nav2_{n}")
}

#[tokio::test(flavor = "multi_thread")]
async fn test_control() {
    let action_name = &action_name();
    let node = test_node();
    let server_requests = node
        .r2r()
        .create_action_server::<FollowJointTrajectory::Action>(&format!(
            "{action_name}/follow_joint_trajectory"
        ))
        .unwrap();
    let publisher = node
        .r2r()
        .create_publisher::<JointTrajectoryControllerState>(
            &format!("{action_name}/state"),
            r2r::QosProfile::default(),
        )
        .unwrap();
    let state = Arc::new(Mutex::new(JointTrajectoryControllerState {
        joint_names: vec!["j1".to_owned(), "j2".to_owned()],
        actual: trajectory_msg::JointTrajectoryPoint {
            positions: vec![0.0; 2],
            ..Default::default()
        },
        ..Default::default()
    }));
    tokio::spawn(test_control_server(
        node.clone(),
        server_requests,
        state.clone(),
    ));
    tokio::spawn(async move {
        loop {
            publisher.publish(&state.lock().unwrap()).unwrap();
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });
    node.run_spin_thread(Duration::from_millis(100));
    let client = Ros2ControlClient::new(node, action_name).unwrap();

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
    node: Node,
    goal: r2r::ActionServerGoal<FollowJointTrajectory::Action>,
    state: Arc<Mutex<JointTrajectoryControllerState>>,
) -> FollowJointTrajectory::Result {
    let mut timer = node // local timer, will be dropped after this request is processed.
        .r2r()
        .create_wall_timer(Duration::from_millis(800))
        .expect("could not create timer");

    state
        .lock()
        .unwrap()
        .actual
        .positions
        .clone_from(&goal.goal.trajectory.points.last().unwrap().positions);

    timer.tick().await.unwrap();

    FollowJointTrajectory::Result::default()
}

async fn test_control_server(
    node: Node,
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
