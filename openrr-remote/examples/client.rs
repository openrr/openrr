use std::time::{Duration, SystemTime};

use arci::{
    Gamepad, JointTrajectoryClient, Localization, MoveBase, Navigation, Speaker, TransformResolver,
};
use openrr_remote::{
    RemoteGamepadSender, RemoteJointTrajectoryClientSender, RemoteLocalizationSender,
    RemoteMoveBaseSender, RemoteNavigationSender, RemoteSpeakerSender,
    RemoteTransformResolverSender,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    const ENDPOINT: &str = "http://[::1]:50051";

    let client = RemoteJointTrajectoryClientSender::connect(ENDPOINT).await?;
    let speaker = RemoteSpeakerSender::connect(ENDPOINT).await?;
    let base = RemoteMoveBaseSender::connect(ENDPOINT).await?;
    let nav = RemoteNavigationSender::connect(ENDPOINT).await?;
    let loc = RemoteLocalizationSender::connect(ENDPOINT).await?;
    let resolver = RemoteTransformResolverSender::connect(ENDPOINT).await?;
    let gamepad = RemoteGamepadSender::connect(ENDPOINT).await?;

    println!("JointTrajectoryClient");
    dbg!(client.joint_names());
    dbg!(client.current_joint_positions()?);
    client
        .send_joint_positions(vec![1.0, 2.0], Duration::from_secs_f64(0.1))?
        .await?;
    client
        .send_joint_trajectory(vec![arci::TrajectoryPoint::new(
            vec![3.0, 4.0],
            Duration::from_secs_f64(0.1),
        )])?
        .await?;

    println!("Speaker");
    speaker.speak("abc")?.await?;

    println!("MoveBase");
    dbg!(base.current_velocity()?);
    base.send_velocity(&arci::BaseVelocity {
        x: 1.0,
        y: 2.0,
        theta: 3.0,
    })?;

    println!("Navigation");
    nav.send_goal_pose(
        arci::Isometry2::new(arci::Vector2::new(1.0, 2.0), 3.0),
        "",
        Duration::from_secs_f64(0.1),
    )?
    .await?;
    nav.cancel()?;

    println!("Localization");
    dbg!(loc.current_pose("")?);

    println!("TransformResolver");
    resolver.resolve_transformation("", "", SystemTime::UNIX_EPOCH)?;

    println!("Gamepad");
    dbg!(gamepad.next_event().await);
    gamepad.stop();

    Ok(())
}
