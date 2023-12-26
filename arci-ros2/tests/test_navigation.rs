mod shared;

use std::{
    sync::atomic::{AtomicUsize, Ordering},
    time::Duration,
};

use arci::*;
use arci_ros2::{msg::nav2_msgs::NavigateToPose, Node, Ros2Navigation};
use assert_approx_eq::assert_approx_eq;
use ros2_client::action;
use shared::*;

fn action_name() -> String {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    let n = COUNT.fetch_add(1, Ordering::Relaxed);
    format!("/test_nav2_{n}")
}

#[flaky_test::flaky_test]
fn test_nav() {
    test_nav_inner();
}
#[tokio::main]
async fn test_nav_inner() {
    let action_name = &action_name();
    let node = test_node();
    let nav = Ros2Navigation::new(node.clone(), action_name);

    start_test_nav_server(&node, action_name);

    // TODO: wait for server
    tokio::time::sleep(Duration::from_secs(1)).await;

    nav.send_goal_pose(
        Isometry2::new(Vector2::new(-0.6, 0.2), 1.0),
        "map",
        Duration::from_secs(80),
    )
    .unwrap()
    .await
    .unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn test_nav_timeout() {
    let action_name = &action_name();
    let node = test_node();
    let nav = Ros2Navigation::new(node.clone(), action_name);

    start_test_nav_server(&node, action_name);

    // TODO: wait for server
    tokio::time::sleep(Duration::from_secs(1)).await;

    assert!(nav
        .send_goal_pose(
            Isometry2::new(Vector2::new(-0.6, 0.2), 1.0),
            "map",
            Duration::from_secs(1),
        )
        .unwrap()
        .await
        .unwrap_err()
        .to_string()
        .contains("timeout"));
}

#[flaky_test::flaky_test]
fn test_nav_cancel() {
    test_nav_cancel_inner();
}
#[tokio::main]
async fn test_nav_cancel_inner() {
    let action_name = &action_name();
    let node = test_node();
    let nav = Ros2Navigation::new(node.clone(), action_name);

    start_test_nav_server(&node, action_name);

    // TODO: wait for server
    tokio::time::sleep(Duration::from_secs(1)).await;

    let wait = nav
        .send_goal_pose(
            Isometry2::new(Vector2::new(-0.6, 0.2), 1.0),
            "map",
            Duration::from_secs(80),
        )
        .unwrap();
    // TODO: remove needs of this sleep
    tokio::time::sleep(Duration::from_secs(1)).await;
    nav.cancel().unwrap();
    // TODO: "canceled" should be an error?
    wait.await.unwrap();
    tokio::time::sleep(Duration::from_secs(10)).await;
}

fn start_test_nav_server(node: &Node, action_name: &str) {
    let action_server = action::AsyncActionServer::new(
        node.create_action_server::<NavigateToPose::Action>(action_name)
            .unwrap(),
    );
    std::thread::spawn(move || {
        let server = test_nav_server(action_server);
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(server)
    });
}

async fn test_nav_server(mut action_server: action::AsyncActionServer<NavigateToPose::Action>) {
    loop {
        let new_goal_handle = action_server.receive_new_goal().await.unwrap();
        let goal = action_server
            .get_new_goal(new_goal_handle.clone())
            .unwrap()
            .clone();
        println!(
            "Got goal request with orientation {:?}, position {:?}",
            goal.pose.pose.orientation, goal.pose.pose.position,
        );
        let accepted_goal = action_server.accept_goal(new_goal_handle).await.unwrap();
        let executing_goal = action_server
            .start_executing_goal(accepted_goal)
            .await
            .unwrap();

        let mut work_timer = tokio::time::interval(Duration::from_secs(1));

        let mut feedback_msg = NavigateToPose::Feedback::default();
        let mut orientation_diff = feedback_msg.current_pose.pose.orientation.clone();
        orientation_diff.w = (goal.pose.pose.orientation.w - orientation_diff.w) / 5.0;
        orientation_diff.x = (goal.pose.pose.orientation.x - orientation_diff.x) / 5.0;
        orientation_diff.y = (goal.pose.pose.orientation.y - orientation_diff.y) / 5.0;
        orientation_diff.z = (goal.pose.pose.orientation.z - orientation_diff.z) / 5.0;
        let mut position_diff = feedback_msg.current_pose.pose.position.clone();
        position_diff.x = (goal.pose.pose.position.x - position_diff.x) / 5.0;
        position_diff.y = (goal.pose.pose.position.y - position_diff.y) / 5.0;
        position_diff.z = (goal.pose.pose.position.z - position_diff.z) / 5.0;

        let mut i = 0;
        let result_status = loop {
            tokio::select! {
                _ = work_timer.tick() => {
                    println!("Sending feedback: {feedback_msg:?}");
                    action_server
                        .publish_feedback(executing_goal.clone(), feedback_msg.clone())
                        .await.unwrap();
                    println!("Publish feedback goal_id={:?}", executing_goal.goal_id());
                    if i == 5 {
                        let current_pose = &feedback_msg.current_pose.pose;
                        assert_approx_eq!(current_pose.orientation.w, goal.pose.pose.orientation.w);
                        assert_approx_eq!(current_pose.orientation.x, goal.pose.pose.orientation.x);
                        assert_approx_eq!(current_pose.orientation.y, goal.pose.pose.orientation.y);
                        assert_approx_eq!(current_pose.orientation.z, goal.pose.pose.orientation.z);
                        assert_approx_eq!(current_pose.position.x, goal.pose.pose.position.x);
                        assert_approx_eq!(current_pose.position.y, goal.pose.pose.position.y);
                        assert_approx_eq!(current_pose.position.z, goal.pose.pose.position.z);
                        println!("Reached goal");
                        break action::GoalEndStatus::Succeeded
                    }
                    i += 1;
                    let current_pose = &mut feedback_msg.current_pose.pose;
                    current_pose.orientation.w += orientation_diff.w;
                    current_pose.orientation.x += orientation_diff.x;
                    current_pose.orientation.y += orientation_diff.y;
                    current_pose.orientation.z += orientation_diff.z;
                    current_pose.position.x += position_diff.x;
                    current_pose.position.y += position_diff.y;
                    current_pose.position.z += position_diff.z;
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
                    NavigateToPose::Result::default(),
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
