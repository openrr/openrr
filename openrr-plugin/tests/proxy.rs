use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use arci::{
    BaseVelocity, DummyLocalization, DummyMoveBase, DummyNavigation, Isometry2, Localization,
    MoveBase, Navigation, Speaker, WaitFuture,
};
use assert_approx_eq::assert_approx_eq;
use nalgebra::Vector2;
use openrr_plugin::{RLocalization, RMoveBase, RNavigation, RSpeaker};

// TODO: move to arci?
#[derive(Debug, Default)]
struct DummySpeaker {
    message: Mutex<String>,
}

impl DummySpeaker {
    fn new() -> Self {
        Self::default()
    }

    fn last_message(&self) -> String {
        self.message.lock().unwrap().clone()
    }
}

impl Speaker for DummySpeaker {
    fn speak(&self, message: &str) -> Result<WaitFuture<'static>, arci::Error> {
        *self.message.lock().unwrap() = message.to_string();
        Ok(WaitFuture::ready())
    }
}

#[tokio::test]
#[ignore]
async fn joint_trajectory_client() {
    todo!()
}

#[tokio::test]
async fn speaker() {
    let speaker = Arc::new(DummySpeaker::new());
    let proxy = RSpeaker::new(speaker.clone());

    assert_eq!(speaker.last_message(), "");
    proxy.speak("abc").unwrap().await.unwrap();
    assert_eq!(speaker.last_message(), "abc");
}

#[tokio::test]
async fn move_base() {
    let base = Arc::new(DummyMoveBase::new());
    let proxy = RMoveBase::new(base.clone());

    let vel1 = base.current_velocity().unwrap();
    assert_approx_eq!(vel1.x, 0.0);
    assert_approx_eq!(vel1.y, 0.0);
    assert_approx_eq!(vel1.theta, 0.0);
    proxy
        .send_velocity(&BaseVelocity::new(0.1, 0.2, -3.0))
        .unwrap();
    let vel2 = base.current_velocity().unwrap();
    assert_approx_eq!(vel2.x, 0.1);
    assert_approx_eq!(vel2.y, 0.2);
    assert_approx_eq!(vel2.theta, -3.0);
}

#[tokio::test]
async fn navigation() {
    let nav = Arc::new(DummyNavigation::new());
    let proxy = RNavigation::new(nav.clone());

    proxy
        .send_goal_pose(
            Isometry2::new(Vector2::new(1.0, 2.0), 3.0),
            "",
            Duration::default(),
        )
        .unwrap()
        .await
        .unwrap();
    let goal_pose2 = nav.current_goal_pose().unwrap();
    assert_approx_eq!(goal_pose2.translation.x, 1.0);
    assert_approx_eq!(goal_pose2.translation.y, 2.0);
    assert_approx_eq!(goal_pose2.rotation.angle(), 3.0);
}

#[tokio::test]
async fn localization() {
    let loc = Arc::new(DummyLocalization::new());
    let proxy = RLocalization::new(loc);

    let pose = proxy.current_pose("").unwrap();
    assert_eq!(pose, pose.inverse()); // only identity mapping satisfies this
}

#[tokio::test]
#[ignore]
async fn transform_resolver() {
    todo!()
}

#[tokio::test]
#[ignore]
async fn gamepad() {
    todo!()
}
