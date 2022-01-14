#![cfg(feature = "ros2")]

use std::{
    sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        Arc,
    },
    time::Duration,
};

use arci::*;
use arci_ros2::{r2r, Ros2Navigation};
use assert_approx_eq::assert_approx_eq;
use futures::{
    future::{self, Either},
    stream::{Stream, StreamExt},
};
use parking_lot::Mutex;
use r2r::nav2_msgs::action::NavigateToPose;

// (node_name, action_name)
fn node_and_action_name() -> (String, String) {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    let n = COUNT.fetch_add(1, Ordering::SeqCst);
    let node_name = format!("test_nav2_node_{n}");
    let action_name = format!("/test_nav2_{n}");
    (node_name, action_name)
}

#[flaky_test::flaky_test]
fn test_nav() {
    test_nav_inner();
}
#[tokio::main(flavor = "current_thread")]
async fn test_nav_inner() {
    let (node_name, action_name) = &node_and_action_name();
    let ctx = r2r::Context::create().unwrap();
    let nav = Ros2Navigation::new(ctx.clone(), action_name);
    let node = Arc::new(Mutex::new(
        r2r::Node::create(ctx, node_name, "arci_ros2").unwrap(),
    ));

    let server_requests = node
        .lock()
        .create_action_server::<NavigateToPose::Action>(action_name)
        .unwrap();

    let node_cb = node.clone();
    tokio::spawn(test_nav_server(node_cb, server_requests));

    std::thread::spawn(move || loop {
        node.lock().spin_once(Duration::from_millis(100));
    });

    nav.send_goal_pose(
        Isometry2::new(Vector2::new(-0.6, 0.2), 1.0),
        "map",
        Duration::from_secs(80),
    )
    .unwrap()
    .await
    .unwrap();
}

#[tokio::test]
async fn test_nav_timeout() {
    let (node_name, action_name) = &node_and_action_name();
    let ctx = r2r::Context::create().unwrap();
    let nav = Ros2Navigation::new(ctx.clone(), action_name);
    let node = Arc::new(Mutex::new(
        r2r::Node::create(ctx, node_name, "arci_ros2").unwrap(),
    ));

    let server_requests = node
        .lock()
        .create_action_server::<NavigateToPose::Action>(action_name)
        .unwrap();

    let node_cb = node.clone();
    tokio::spawn(test_nav_server(node_cb, server_requests));

    std::thread::spawn(move || loop {
        node.lock().spin_once(Duration::from_millis(100));
    });

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
#[tokio::main(flavor = "current_thread")]
async fn test_nav_cancel_inner() {
    let (node_name, action_name) = &node_and_action_name();
    let ctx = r2r::Context::create().unwrap();
    let nav = Ros2Navigation::new(ctx.clone(), action_name);
    let node = Arc::new(Mutex::new(
        r2r::Node::create(ctx, node_name, "arci_ros2").unwrap(),
    ));

    let server_requests = node
        .lock()
        .create_action_server::<NavigateToPose::Action>(action_name)
        .unwrap();

    let node_cb = node.clone();
    tokio::spawn(test_nav_server(node_cb, server_requests));

    std::thread::spawn(move || loop {
        node.lock().spin_once(Duration::from_millis(100));
    });

    let wait = nav
        .send_goal_pose(
            Isometry2::new(Vector2::new(-0.6, 0.2), 1.0),
            "map",
            Duration::from_secs(80),
        )
        .unwrap();
    // TODO: remove needs of this sleep
    tokio::time::sleep(Duration::from_millis(1000)).await;
    nav.cancel().unwrap();
    // TODO: "canceled" should be an error?
    wait.await.unwrap();
}

async fn run_goal(
    node: Arc<Mutex<r2r::Node>>,
    g: r2r::ActionServerGoal<NavigateToPose::Action>,
    canceled: Arc<AtomicBool>,
) -> NavigateToPose::Result {
    let mut timer = node // local timer, will be dropped after this request is processed.
        .lock()
        .create_wall_timer(Duration::from_millis(800))
        .expect("could not create timer");

    let mut feedback_msg = NavigateToPose::Feedback::default();
    g.publish_feedback(feedback_msg.clone()).expect("fail");

    let mut orientation_diff = feedback_msg.current_pose.pose.orientation.clone();
    orientation_diff.w = (g.goal.pose.pose.orientation.w - orientation_diff.w) / 5.0;
    orientation_diff.x = (g.goal.pose.pose.orientation.x - orientation_diff.x) / 5.0;
    orientation_diff.y = (g.goal.pose.pose.orientation.y - orientation_diff.y) / 5.0;
    orientation_diff.z = (g.goal.pose.pose.orientation.z - orientation_diff.z) / 5.0;
    let mut position_diff = feedback_msg.current_pose.pose.position.clone();
    position_diff.x = (g.goal.pose.pose.position.x - position_diff.x) / 5.0;
    position_diff.y = (g.goal.pose.pose.position.y - position_diff.y) / 5.0;
    position_diff.z = (g.goal.pose.pose.position.z - position_diff.z) / 5.0;
    for _ in 0..5 {
        if canceled.load(Ordering::SeqCst) {
            break;
        }
        let current_pose = &mut feedback_msg.current_pose.pose;
        current_pose.orientation.w += orientation_diff.w;
        current_pose.orientation.x += orientation_diff.x;
        current_pose.orientation.y += orientation_diff.y;
        current_pose.orientation.z += orientation_diff.z;
        current_pose.position.x += position_diff.x;
        current_pose.position.y += position_diff.y;
        current_pose.position.z += position_diff.z;
        g.publish_feedback(feedback_msg.clone()).expect("fail");
        println!("Sending feedback: {feedback_msg:?}");
        timer.tick().await.unwrap();
    }

    if !canceled.load(Ordering::SeqCst) {
        let current_pose = &feedback_msg.current_pose.pose;
        assert_approx_eq!(current_pose.orientation.w, g.goal.pose.pose.orientation.w);
        assert_approx_eq!(current_pose.orientation.x, g.goal.pose.pose.orientation.x);
        assert_approx_eq!(current_pose.orientation.y, g.goal.pose.pose.orientation.y);
        assert_approx_eq!(current_pose.orientation.z, g.goal.pose.pose.orientation.z);
        assert_approx_eq!(current_pose.position.x, g.goal.pose.pose.position.x);
        assert_approx_eq!(current_pose.position.y, g.goal.pose.pose.position.y);
        assert_approx_eq!(current_pose.position.z, g.goal.pose.pose.position.z);
    }

    NavigateToPose::Result {
        result: r2r::std_msgs::msg::Empty {},
    }
}

async fn test_nav_server(
    node: Arc<Mutex<r2r::Node>>,
    mut requests: impl Stream<Item = r2r::ActionServerGoalRequest<NavigateToPose::Action>> + Unpin,
) {
    while let Some(req) = requests.next().await {
        println!(
            "Got goal request with orientation {:?}, position {:?}, goal id: {}",
            req.goal.pose.pose.orientation, req.goal.pose.pose.position, req.uuid
        );
        let (mut g, mut cancel) = req.accept().expect("could not accept goal");

        let canceled = Arc::new(AtomicBool::new(false));
        let goal_fut = tokio::task::spawn(run_goal(node.clone(), g.clone(), canceled.clone()));

        match future::select(goal_fut, cancel.next()).await {
            Either::Left((result, _)) => {
                let result = result.unwrap();
                g.succeed(result).expect("could not send result");
            }
            Either::Right((request, _)) => {
                if let Some(request) = request {
                    println!("got cancel request: {}", request.uuid);
                    request.accept();
                    canceled.store(true, Ordering::SeqCst);
                    g.cancel(NavigateToPose::Result {
                        result: r2r::std_msgs::msg::Empty {},
                    })
                    .unwrap();
                }
            }
        }
    }
}
