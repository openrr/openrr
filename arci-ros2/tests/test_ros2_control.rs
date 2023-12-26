mod shared;

use std::{
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc, Mutex,
    },
    time::Duration,
};

use arci::*;
use arci_ros2::{
    msg::{
        control_msgs::{FollowJointTrajectory, JointTrajectoryControllerState},
        trajectory_msgs,
    },
    Node, Ros2ControlClient,
};
use ros2_client::action;
use shared::*;

fn action_name() -> String {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    let n = COUNT.fetch_add(1, Ordering::Relaxed);
    format!("/test_ros2_control_{n}")
}

#[flaky_test::flaky_test]
fn test_control() {
    test_control_inner();
}
#[tokio::main]
async fn test_control_inner() {
    let action_name = &action_name();
    let node = test_node();
    let state_topic = node
        .create_topic::<JointTrajectoryControllerState>(&format!("{action_name}/state"))
        .unwrap();
    let publisher = node
        .create_publisher::<JointTrajectoryControllerState>(&state_topic)
        .unwrap();
    let state = Arc::new(Mutex::new(JointTrajectoryControllerState {
        joint_names: vec!["j1".to_owned(), "j2".to_owned()],
        actual: trajectory_msgs::JointTrajectoryPoint {
            positions: vec![0.0; 2],
            ..Default::default()
        },
        ..Default::default()
    }));
    start_test_control_server(&node, action_name, state.clone()).await;
    tokio::spawn(async move {
        loop {
            let new = state.lock().unwrap().clone();
            publisher.publish(new).unwrap();
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
    });
    let client = Ros2ControlClient::new(node, action_name).unwrap();

    assert_eq!(client.joint_names(), vec!["j1".to_owned(), "j2".to_owned()]);
    assert_eq!(client.current_joint_positions().unwrap(), vec![0.0, 0.0]);
    client
        .send_joint_positions(vec![1.0, 0.5], Duration::from_secs(20))
        .unwrap()
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_secs(1)).await;
    assert_eq!(client.current_joint_positions().unwrap(), vec![1.0, 0.5]);
}

async fn start_test_control_server(
    node: &Node,
    action_name: &str,
    state: Arc<Mutex<JointTrajectoryControllerState>>,
) {
    let action_server = action::AsyncActionServer::new(
        node.create_action_server::<FollowJointTrajectory::Action>(&format!(
            "{action_name}/follow_joint_trajectory"
        ))
        .unwrap(),
    );
    std::thread::spawn(move || {
        let server = test_control_server(action_server, state);
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(server)
    });

    // TODO: wait for server
    tokio::time::sleep(Duration::from_secs(1)).await;
}

async fn test_control_server(
    mut action_server: action::AsyncActionServer<FollowJointTrajectory::Action>,
    state: Arc<Mutex<JointTrajectoryControllerState>>,
) {
    loop {
        let new_goal_handle = action_server.receive_new_goal().await.unwrap();
        let goal = action_server
            .get_new_goal(new_goal_handle.clone())
            .unwrap()
            .clone();
        println!("Got goal request with trajectory {:?}", goal.trajectory);
        let accepted_goal = action_server.accept_goal(new_goal_handle).await.unwrap();
        let executing_goal = action_server
            .start_executing_goal(accepted_goal)
            .await
            .unwrap();

        let mut work_timer = tokio::time::interval(Duration::from_secs(1));

        let result_status = loop {
            tokio::select! {
                _ = work_timer.tick() => {
                    // TODO
                    // println!("Sending feedback: {feedback_msg:?}");
                    // action_server
                    //     .publish_feedback(executing_goal.clone(), feedback_msg.clone())
                    //     .await.unwrap();
                    // println!("Publish feedback goal_id={:?}", executing_goal.goal_id());
                    let last = goal
                        .trajectory
                        .points
                        .last()
                        .unwrap();
                    state.lock().unwrap().actual.positions = last
                        .positions
                        .clone();
                    tokio::time::sleep(Duration::new(
                        last.time_from_start.sec as _,
                        last.time_from_start.nanosec,
                    )).await;
                    println!("Reached goal");
                    break action::GoalEndStatus::Succeeded
                },
                cancel_handle = action_server.receive_cancel_request() => {
                    let cancel_handle = cancel_handle.unwrap();
                    let my_goal = executing_goal.goal_id();
                    if cancel_handle.contains_goal(&my_goal) {
                        println!("Got cancel request!");
                        action_server
                            .respond_to_cancel_requests(&cancel_handle, std::iter::once(my_goal))
                            .await
                            .unwrap();
                        break action::GoalEndStatus::Canceled
                    } else {
                        println!("Received a cancel request for some other goals.");
                    }
                }
            }
        };
        // We must return a result in all cases
        // Also add a timeout in case client does not request a result.
        let timeout = tokio::time::sleep(Duration::from_secs(10));
        tokio::select! {
            res = action_server
                .send_result_response(
                    executing_goal,
                    result_status,
                    FollowJointTrajectory::Result::default(),
                ) => {
                if let Err(e) = res {
                    println!("Error: Cannot send result response {e:?}");
                }
            }
            _ = timeout => println!("Error: Cannot send result response: timeout"),
        }
        println!("Goal ended. Reason={result_status:?}");
    }
}
