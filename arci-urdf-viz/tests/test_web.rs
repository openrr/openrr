use std::{f64, time::Duration};

use arci::{nalgebra, Localization, MoveBase, Navigation};
use arci_urdf_viz::*;
use assert_approx_eq::assert_approx_eq;
use urdf_viz::WebServer;
use url::Url;

#[easy_ext::ext]
impl WebServer {
    fn start_background(self) {
        let handle = self.handle();
        std::thread::spawn(move || self.start().unwrap());
        std::thread::spawn(move || loop {
            if let Some(positions) = handle.take_target_joint_positions() {
                *handle.current_joint_positions() = positions;
            }
            if let Some(origin) = handle.take_target_robot_origin() {
                *handle.current_robot_origin() = origin;
            }
        });
        std::thread::sleep(Duration::from_secs(1)); // Wait for web server to start.
    }
}

#[test]
fn test_set_get_vel() {
    const PORT: u16 = 8888;
    let web_server = WebServer::new(PORT, Default::default());
    web_server.start_background();
    let c =
        UrdfVizWebClient::new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap()).unwrap();
    c.run_send_velocity_thread();
    let v = c.current_velocity().unwrap();
    assert_approx_eq!(v.x, 0.0);
    assert_approx_eq!(v.y, 0.0);
    assert_approx_eq!(v.theta, 0.0);
    c.send_velocity(&arci::BaseVelocity::new(1.0, 2.0, 3.0))
        .unwrap();
    let v = c.current_velocity().unwrap();
    assert_approx_eq!(v.x, 1.0);
    assert_approx_eq!(v.y, 2.0);
    assert_approx_eq!(v.theta, 3.0);
}

#[tokio::test]
async fn test_set_get_pose() {
    const PORT: u16 = 8889;
    let web_server = WebServer::new(PORT, Default::default());
    web_server.start_background();
    let c =
        UrdfVizWebClient::new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap()).unwrap();
    let pose = c.current_pose("").unwrap();
    assert_approx_eq!(pose.translation.x, 0.0);
    assert_approx_eq!(pose.translation.y, 0.0);
    assert_approx_eq!(pose.rotation.angle(), f64::consts::PI);
    c.send_goal_pose(
        nalgebra::Isometry2::new(nalgebra::Vector2::new(1.0, 2.0), 3.0),
        "",
        Duration::from_secs(0),
    )
    .unwrap()
    .await
    .unwrap();
    std::thread::sleep(Duration::from_millis(5));
    let pose = c.current_pose("").unwrap();
    assert_approx_eq!(pose.translation.x, 1.0);
    assert_approx_eq!(pose.translation.y, 2.0);
    assert_approx_eq!(pose.rotation.angle(), 3.0);
}

#[test]
fn test_set_get_pose_no_wait() {
    const PORT: u16 = 8890;
    let web_server = WebServer::new(PORT, Default::default());
    web_server.start_background();
    let c =
        UrdfVizWebClient::new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap()).unwrap();
    let pose = c.current_pose("").unwrap();
    assert_approx_eq!(pose.translation.x, 0.0);
    assert_approx_eq!(pose.translation.y, 0.0);
    assert_approx_eq!(pose.rotation.angle(), f64::consts::PI);
    let _ = c
        .send_goal_pose(
            nalgebra::Isometry2::new(nalgebra::Vector2::new(1.0, 2.0), 3.0),
            "",
            Duration::from_secs(0),
        )
        .unwrap();
    std::thread::sleep(Duration::from_millis(5));
    let pose = c.current_pose("").unwrap();
    assert_approx_eq!(pose.translation.x, 1.0);
    assert_approx_eq!(pose.translation.y, 2.0);
    assert_approx_eq!(pose.rotation.angle(), 3.0);
}
