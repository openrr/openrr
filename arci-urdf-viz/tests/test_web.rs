mod util;

use std::{f64, time::Duration};

use arci::{nalgebra, Localization, MoveBase, Navigation};
use arci_urdf_viz::*;
use assert_approx_eq::assert_approx_eq;
use urdf_viz::WebServer;
use util::*;

#[test]
fn test_set_get_vel() {
    let (port, url) = port_and_url();
    let web_server = WebServer::new(port, Default::default());
    web_server.start_background();
    let c = UrdfVizWebClient::new(url).unwrap();
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

#[flaky_test::flaky_test(tokio)]
async fn test_set_get_pose() {
    let (port, url) = port_and_url();
    let web_server = WebServer::new(port, Default::default());
    web_server.start_background();
    let c = UrdfVizWebClient::new(url).unwrap();
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
    std::thread::sleep(Duration::from_millis(10));
    let pose = c.current_pose("").unwrap();
    assert_approx_eq!(pose.translation.x, 1.0);
    assert_approx_eq!(pose.translation.y, 2.0);
    assert_approx_eq!(pose.rotation.angle(), 3.0);
}

#[flaky_test::flaky_test]
fn test_set_get_pose_no_wait() {
    let (port, url) = port_and_url();
    let web_server = WebServer::new(port, Default::default());
    web_server.start_background();
    let c = UrdfVizWebClient::new(url).unwrap();
    let pose = c.current_pose("").unwrap();
    assert_approx_eq!(pose.translation.x, 0.0);
    assert_approx_eq!(pose.translation.y, 0.0);
    assert_approx_eq!(pose.rotation.angle(), f64::consts::PI);
    drop(
        c.send_goal_pose(
            nalgebra::Isometry2::new(nalgebra::Vector2::new(1.0, 2.0), 3.0),
            "",
            Duration::from_secs(0),
        )
        .unwrap(),
    );
    std::thread::sleep(Duration::from_millis(10));
    let pose = c.current_pose("").unwrap();
    assert_approx_eq!(pose.translation.x, 1.0);
    assert_approx_eq!(pose.translation.y, 2.0);
    assert_approx_eq!(pose.rotation.angle(), 3.0);
}
