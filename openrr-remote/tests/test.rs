use std::{
    net::SocketAddr,
    sync::{
        atomic::{AtomicU16, Ordering},
        Arc,
    },
    time::{Duration, SystemTime},
};

use anyhow::Result;
use arci::{
    gamepad::GamepadEvent, BaseVelocity, DummyGamepad, DummyJointTrajectoryClient,
    DummyLocalization, DummyMoveBase, DummyNavigation, DummySpeaker, DummyTransformResolver,
    Gamepad, Isometry2, JointTrajectoryClient, Localization, MoveBase, Navigation, Speaker,
    TrajectoryPoint, TransformResolver, Vector2,
};
use assert_approx_eq::assert_approx_eq;
use openrr_remote::{
    RemoteGamepadReceiver, RemoteGamepadSender, RemoteJointTrajectoryClientReceiver,
    RemoteJointTrajectoryClientSender, RemoteLocalizationReceiver, RemoteLocalizationSender,
    RemoteMoveBaseReceiver, RemoteMoveBaseSender, RemoteNavigationReceiver, RemoteNavigationSender,
    RemoteSpeakerReceiver, RemoteSpeakerSender, RemoteTransformResolverReceiver,
    RemoteTransformResolverSender,
};

fn endpoint() -> (SocketAddr, String) {
    static PORT: AtomicU16 = AtomicU16::new(50061);
    let addr = format!("[::1]:{}", PORT.fetch_add(1, Ordering::SeqCst));
    (addr.parse().unwrap(), format!("http://{}", addr))
}

#[tokio::test(flavor = "multi_thread")]
async fn joint_trajectory_client() -> Result<()> {
    let (addr, endpoint) = endpoint();

    // Launch server
    {
        let client =
            RemoteJointTrajectoryClientReceiver::new(DummyJointTrajectoryClient::new(vec![
                "a".to_owned()
            ]));
        tokio::spawn(client.serve(addr));
        tokio::time::sleep(Duration::from_secs(1)).await;
    }

    let client = RemoteJointTrajectoryClientSender::connect(endpoint).await?;
    assert_eq!(client.joint_names(), vec!["a".to_owned()]);
    assert_eq!(client.current_joint_positions()?, vec![0.0]);
    client
        .send_joint_positions(vec![1.0], Duration::from_secs_f64(0.1))?
        .await?;
    assert_eq!(client.current_joint_positions()?, vec![1.0]);
    client
        .send_joint_trajectory(vec![TrajectoryPoint::new(
            vec![2.0],
            Duration::from_secs_f64(0.1),
        )])?
        .await?;
    assert_eq!(client.current_joint_positions()?, vec![2.0]);
    client
        .send_joint_trajectory(vec![TrajectoryPoint {
            positions: vec![3.0],
            velocities: Some(vec![3.0]),
            time_from_start: Duration::from_secs_f64(0.1),
        }])?
        .await?;
    assert_eq!(client.current_joint_positions()?, vec![3.0]);

    // no wait
    let _ = client.send_joint_positions(vec![4.0], Duration::from_secs_f64(0.1))?;
    tokio::time::sleep(Duration::from_secs_f64(0.5)).await;
    assert_eq!(client.current_joint_positions()?, vec![4.0]);
    let _ = client.send_joint_trajectory(vec![TrajectoryPoint::new(
        vec![5.0],
        Duration::from_secs_f64(0.1),
    )])?;
    tokio::time::sleep(Duration::from_secs_f64(0.5)).await;
    assert_eq!(client.current_joint_positions()?, vec![5.0]);

    Ok(())
}

#[tokio::test(flavor = "multi_thread")]
async fn speaker() -> Result<()> {
    let (addr, endpoint) = endpoint();

    let recv_speaker = Arc::new(DummySpeaker::new());
    // Launch server
    {
        let speaker = RemoteSpeakerReceiver::new(recv_speaker.clone());
        tokio::spawn(speaker.serve(addr));
        tokio::time::sleep(Duration::from_secs(1)).await;
    }

    let speaker = RemoteSpeakerSender::connect(endpoint).await?;
    assert_eq!(recv_speaker.current_message(), "");
    speaker.speak("abc")?.await?;
    assert_eq!(recv_speaker.current_message(), "abc");

    Ok(())
}

#[tokio::test(flavor = "multi_thread")]
async fn move_base() -> Result<()> {
    let (addr, endpoint) = endpoint();

    // Launch server
    {
        let base = RemoteMoveBaseReceiver::new(DummyMoveBase::new());
        tokio::spawn(base.serve(addr));
        tokio::time::sleep(Duration::from_secs(1)).await;
    }

    let base = RemoteMoveBaseSender::connect(endpoint).await?;
    let v = base.current_velocity()?;
    assert_approx_eq!(v.x, 0.0);
    assert_approx_eq!(v.y, 0.0);
    assert_approx_eq!(v.theta, 0.0);
    base.send_velocity(&BaseVelocity {
        x: 1.0,
        y: 2.0,
        theta: 3.0,
    })?;
    let v = base.current_velocity()?;
    assert_approx_eq!(v.x, 1.0);
    assert_approx_eq!(v.y, 2.0);
    assert_approx_eq!(v.theta, 3.0);

    Ok(())
}

#[tokio::test(flavor = "multi_thread")]
async fn navigation() -> Result<()> {
    let (addr, endpoint) = endpoint();

    let recv_nav = Arc::new(DummyNavigation::new());
    // Launch server
    {
        let nav = RemoteNavigationReceiver::new(recv_nav.clone());
        tokio::spawn(nav.serve(addr));
        tokio::time::sleep(Duration::from_secs(1)).await;
    }

    let nav = RemoteNavigationSender::connect(endpoint).await?;
    nav.send_goal_pose(
        Isometry2::new(Vector2::new(1.0, 2.0), 3.0),
        "",
        Duration::default(),
    )?
    .await?;
    let pose = recv_nav.current_goal_pose()?;
    assert_approx_eq!(pose.translation.x, 1.0);
    assert_approx_eq!(pose.translation.y, 2.0);
    assert_approx_eq!(pose.rotation.angle(), 3.0);
    nav.cancel()?;
    assert!(recv_nav.is_canceled());

    Ok(())
}

#[tokio::test(flavor = "multi_thread")]
async fn localization() -> Result<()> {
    let (addr, endpoint) = endpoint();

    // Launch server
    {
        let loc = RemoteLocalizationReceiver::new(DummyLocalization::new());
        tokio::spawn(loc.serve(addr));
        tokio::time::sleep(Duration::from_secs(1)).await;
    }

    let loc = RemoteLocalizationSender::connect(endpoint).await?;
    let pose = loc.current_pose("")?;
    assert_eq!(pose, pose.inverse()); // only identity mapping satisfies this

    Ok(())
}

#[tokio::test(flavor = "multi_thread")]
async fn transform_resolver() -> Result<()> {
    let (addr, endpoint) = endpoint();

    let recv_resolver = Arc::new(DummyTransformResolver::default());
    // Launch server
    {
        let resolver = RemoteTransformResolverReceiver::new(recv_resolver.clone());
        tokio::spawn(resolver.serve(addr));
        tokio::time::sleep(Duration::from_secs(1)).await;
    }

    let resolver = RemoteTransformResolverSender::connect(endpoint).await?;
    let transformation = resolver.resolve_transformation("", "", SystemTime::UNIX_EPOCH)?;
    assert_eq!(transformation, transformation.inverse());

    Ok(())
}

#[tokio::test(flavor = "multi_thread")]
async fn gamepad() -> Result<()> {
    let (addr, endpoint) = endpoint();

    let recv_gamepad = Arc::new(DummyGamepad::with_all_events());
    // Launch server
    {
        let gamepad = RemoteGamepadReceiver::new(recv_gamepad.clone());
        tokio::spawn(gamepad.serve(addr));
        tokio::time::sleep(Duration::from_secs(1)).await;
    }

    let gamepad = RemoteGamepadSender::connect(endpoint).await?;
    for expected in &recv_gamepad.events {
        match (expected, gamepad.next_event().await) {
            (GamepadEvent::ButtonPressed(expected), GamepadEvent::ButtonPressed(actual)) => {
                assert_eq!(*expected, actual);
            }
            (GamepadEvent::ButtonReleased(expected), GamepadEvent::ButtonReleased(actual)) => {
                assert_eq!(*expected, actual);
            }
            (
                GamepadEvent::AxisChanged(expected_axis, expected_val),
                GamepadEvent::AxisChanged(actual_axis, actual_val),
            ) => {
                assert_eq!(*expected_axis, actual_axis);
                assert_approx_eq!(*expected_val, actual_val);
            }
            (GamepadEvent::Unknown, GamepadEvent::Unknown) => {}
            (expected, actual) => panic!(
                "event mismatch, expected: {:?}, actual: {:?}",
                expected, actual
            ),
        }
    }
    gamepad.stop();
    assert!(recv_gamepad.is_stopped());

    Ok(())
}

#[tokio::test(flavor = "multi_thread")]
async fn multiple() -> Result<()> {
    let (addr, endpoint) = endpoint();

    let recv_speaker = Arc::new(DummySpeaker::new());
    let recv_nav = Arc::new(DummyNavigation::new());
    let recv_resolver = Arc::new(DummyTransformResolver::default());
    let recv_gamepad = Arc::new(DummyGamepad::with_all_events());
    // Launch server
    {
        let client =
            RemoteJointTrajectoryClientReceiver::new(DummyJointTrajectoryClient::new(vec![
                "a".to_owned()
            ]));
        let speaker = RemoteSpeakerReceiver::new(recv_speaker.clone());
        let base = RemoteMoveBaseReceiver::new(DummyMoveBase::new());
        let nav = RemoteNavigationReceiver::new(recv_nav.clone());
        let loc = RemoteLocalizationReceiver::new(DummyLocalization::new());
        let resolver = RemoteTransformResolverReceiver::new(recv_resolver.clone());
        let gamepad = RemoteGamepadReceiver::new(recv_gamepad.clone());
        tokio::spawn(
            tonic::transport::Server::builder()
                .add_service(client.into_service())
                .add_service(speaker.into_service())
                .add_service(base.into_service())
                .add_service(nav.into_service())
                .add_service(loc.into_service())
                .add_service(resolver.into_service())
                .add_service(gamepad.into_service())
                .serve(addr),
        );
        tokio::time::sleep(Duration::from_secs(1)).await;
    }

    let client = RemoteJointTrajectoryClientSender::connect(endpoint.clone()).await?;
    let speaker = RemoteSpeakerSender::connect(endpoint.clone()).await?;
    let base = RemoteMoveBaseSender::connect(endpoint.clone()).await?;
    let nav = RemoteNavigationSender::connect(endpoint.clone()).await?;
    let loc = RemoteLocalizationSender::connect(endpoint.clone()).await?;
    let resolver = RemoteTransformResolverSender::connect(endpoint.clone()).await?;
    let gamepad = RemoteGamepadSender::connect(endpoint).await?;

    assert_eq!(client.joint_names(), vec!["a".to_owned()]);
    assert_approx_eq!(client.current_joint_positions()?[0], 0.0);
    client
        .send_joint_positions(vec![1.0], Duration::from_secs_f64(0.1))?
        .await?;
    assert_approx_eq!(client.current_joint_positions()?[0], 1.0);
    client
        .send_joint_trajectory(vec![TrajectoryPoint::new(
            vec![2.0],
            Duration::from_secs_f64(0.1),
        )])?
        .await?;
    assert_approx_eq!(client.current_joint_positions()?[0], 2.0);

    let v = base.current_velocity()?;
    assert_approx_eq!(v.x, 0.0);
    assert_approx_eq!(v.y, 0.0);
    assert_approx_eq!(v.theta, 0.0);
    base.send_velocity(&BaseVelocity {
        x: 1.0,
        y: 2.0,
        theta: 3.0,
    })?;
    let v = base.current_velocity()?;
    assert_approx_eq!(v.x, 1.0);
    assert_approx_eq!(v.y, 2.0);
    assert_approx_eq!(v.theta, 3.0);

    speaker.speak("abc")?.await?;
    assert_eq!(recv_speaker.current_message(), "abc");

    nav.send_goal_pose(
        Isometry2::new(Vector2::new(1.0, 2.0), 3.0),
        "",
        Duration::default(),
    )?
    .await?;
    let pose = recv_nav.current_goal_pose()?;
    assert_approx_eq!(pose.translation.x, 1.0);
    assert_approx_eq!(pose.translation.y, 2.0);
    assert_approx_eq!(pose.rotation.angle(), 3.0);

    let pose = loc.current_pose("")?;
    assert_eq!(pose, pose.inverse()); // only identity mapping satisfies this
    nav.cancel()?;
    assert!(recv_nav.is_canceled());

    let transformation = resolver.resolve_transformation("", "", SystemTime::UNIX_EPOCH)?;
    assert_eq!(transformation, transformation.inverse());

    for expected in &recv_gamepad.events {
        match (expected, gamepad.next_event().await) {
            (GamepadEvent::ButtonPressed(expected), GamepadEvent::ButtonPressed(actual)) => {
                assert_eq!(*expected, actual);
            }
            (GamepadEvent::ButtonReleased(expected), GamepadEvent::ButtonReleased(actual)) => {
                assert_eq!(*expected, actual);
            }
            (
                GamepadEvent::AxisChanged(expected_axis, expected_val),
                GamepadEvent::AxisChanged(actual_axis, actual_val),
            ) => {
                assert_eq!(*expected_axis, actual_axis);
                assert_approx_eq!(*expected_val, actual_val);
            }
            (GamepadEvent::Unknown, GamepadEvent::Unknown) => {}
            (expected, actual) => panic!(
                "event mismatch, expected: {:?}, actual: {:?}",
                expected, actual
            ),
        }
    }
    gamepad.stop();
    assert!(recv_gamepad.is_stopped());

    Ok(())
}
