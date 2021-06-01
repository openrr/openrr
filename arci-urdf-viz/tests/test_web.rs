mod web_server;
use arci::{Localization, MoveBase, Navigation};
use arci_urdf_viz::*;
use assert_approx_eq::assert_approx_eq;
use url::Url;
use web_server::*;

#[test]
fn test_set_get_vel() {
    const PORT: u16 = 8888;
    let web_server = WebServer::new(PORT);
    web_server.start_background();
    let c = UrdfVizWebClient::try_new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap())
        .unwrap();
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
    let web_server = WebServer::new(PORT);
    web_server.start_background();
    let c = UrdfVizWebClient::try_new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap())
        .unwrap();
    let pose = c.current_pose("").unwrap();
    assert_approx_eq!(pose.translation.x, 0.0);
    assert_approx_eq!(pose.translation.y, 0.0);
    assert_approx_eq!(pose.rotation.angle(), 0.0);
    c.send_goal_pose(
        nalgebra::Isometry2::new(nalgebra::Vector2::new(1.0, 2.0), 3.0),
        "",
        std::time::Duration::from_secs(0),
    )
    .unwrap()
    .await
    .unwrap();
    let pose = c.current_pose("").unwrap();
    assert_approx_eq!(pose.translation.x, 1.0);
    assert_approx_eq!(pose.translation.y, 2.0);
    assert_approx_eq!(pose.rotation.angle(), 3.0);
}

#[test]
fn test_set_get_pose_no_wait() {
    const PORT: u16 = 8890;
    let web_server = WebServer::new(PORT);
    web_server.start_background();
    let c = UrdfVizWebClient::try_new(Url::parse(&format!("http://127.0.0.1:{}", PORT)).unwrap())
        .unwrap();
    let pose = c.current_pose("").unwrap();
    assert_approx_eq!(pose.translation.x, 0.0);
    assert_approx_eq!(pose.translation.y, 0.0);
    assert_approx_eq!(pose.rotation.angle(), 0.0);
    let _ = c
        .send_goal_pose(
            nalgebra::Isometry2::new(nalgebra::Vector2::new(1.0, 2.0), 3.0),
            "",
            std::time::Duration::from_secs(0),
        )
        .unwrap();
    let pose = c.current_pose("").unwrap();
    assert_approx_eq!(pose.translation.x, 1.0);
    assert_approx_eq!(pose.translation.y, 2.0);
    assert_approx_eq!(pose.rotation.angle(), 3.0);
}
