use openrr_remote::{
    RemoteGamepadReceiver, RemoteJointTrajectoryClientReceiver, RemoteLocalizationReceiver,
    RemoteMoveBaseReceiver, RemoteNavigationReceiver, RemoteSpeakerReceiver,
    RemoteTransformResolverReceiver,
};

#[derive(Default)]
struct DebugJointTrajectoryClient {}

impl arci::JointTrajectoryClient for DebugJointTrajectoryClient {
    fn joint_names(&self) -> Vec<String> {
        println!("Server received JointTrajectoryClient::joint_names");
        vec!["a".into(), "b".into()]
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        println!("Server received JointTrajectoryClient::current_joint_positions");
        Ok(vec![0.0, 0.0])
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<arci::WaitFuture, arci::Error> {
        println!(
            "Server received JointTrajectoryClient::send_joint_positions (position: {:?}, duration: {:?})",
            positions, duration
        );
        Ok(arci::WaitFuture::ready())
    }

    fn send_joint_trajectory(
        &self,
        trajectory: Vec<arci::TrajectoryPoint>,
    ) -> Result<arci::WaitFuture, arci::Error> {
        println!(
            "Server received JointTrajectoryClient::send_joint_trajectory (trajectory: {:?})",
            trajectory
        );
        Ok(arci::WaitFuture::ready())
    }
}

#[derive(Default)]
struct DebugSpeaker {}

impl arci::Speaker for DebugSpeaker {
    fn speak(&self, message: &str) -> Result<arci::WaitFuture, arci::Error> {
        println!("Server received Speaker::speak (message: {:?})", message);
        Ok(arci::WaitFuture::ready())
    }
}

#[derive(Default)]
struct DebugMoveBase {}

impl arci::MoveBase for DebugMoveBase {
    fn send_velocity(&self, velocity: &arci::BaseVelocity) -> Result<(), arci::Error> {
        println!(
            "Server received MoveBase::send_velocity (velocity: {:?})",
            velocity
        );
        Ok(())
    }

    fn current_velocity(&self) -> Result<arci::BaseVelocity, arci::Error> {
        println!("Server received MoveBase::current_velocity");
        Ok(arci::BaseVelocity::default())
    }
}

#[derive(Default)]
struct DebugNavigation {}

impl arci::Navigation for DebugNavigation {
    fn send_goal_pose(
        &self,
        goal: arci::Isometry2<f64>,
        frame_id: &str,
        timeout: std::time::Duration,
    ) -> Result<arci::WaitFuture, arci::Error> {
        println!(
            "Server received Navigation::send_goal_pose (goal: {:?}, frame_id: {:?}, timeout: {:?})",
            goal, frame_id, timeout
        );
        Ok(arci::WaitFuture::ready())
    }

    fn cancel(&self) -> Result<(), arci::Error> {
        println!("Server received Navigation::cancel");
        Ok(())
    }
}

#[derive(Default)]
struct DebugLocalization {}

impl arci::Localization for DebugLocalization {
    fn current_pose(&self, frame_id: &str) -> Result<arci::Isometry2<f64>, arci::Error> {
        println!(
            "Server received Localization::current_pose (frame_id: {:?})",
            frame_id
        );
        Ok(arci::Isometry2::new(arci::Vector2::new(0.0, 0.0), 0.0))
    }
}

#[derive(Default)]
struct DebugTransformResolver {}

impl arci::TransformResolver for DebugTransformResolver {
    fn resolve_transformation(
        &self,
        from: &str,
        to: &str,
        time: std::time::SystemTime,
    ) -> Result<arci::Isometry3<f64>, arci::Error> {
        println!(
            "Server received TransformResolver::resolve_transformation (from: {:?}, to: {:?}, time: {:?})",
            from, to, time
        );
        Ok(arci::Isometry3::new(
            arci::Vector3::new(0.0, 0.0, 0.0),
            arci::Vector3::new(0.0, 0.0, 0.0),
        ))
    }
}

#[derive(Default)]
struct DebugGamepad {}

#[arci::async_trait]
impl arci::Gamepad for DebugGamepad {
    async fn next_event(&self) -> arci::gamepad::GamepadEvent {
        println!("Server received Gamepad::next_event",);
        arci::gamepad::GamepadEvent::Unknown
    }

    fn stop(&self) {
        println!("Server received Gamepad::stop");
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let addr = "[::1]:50051".parse()?;
    println!("Server listening on {}", addr);

    let client = RemoteJointTrajectoryClientReceiver::new(DebugJointTrajectoryClient::default());
    let speaker = RemoteSpeakerReceiver::new(DebugSpeaker::default());
    let base = RemoteMoveBaseReceiver::new(DebugMoveBase::default());
    let nav = RemoteNavigationReceiver::new(DebugNavigation::default());
    let loc = RemoteLocalizationReceiver::new(DebugLocalization::default());
    let resolver = RemoteTransformResolverReceiver::new(DebugTransformResolver::default());
    let gamepad = RemoteGamepadReceiver::new(DebugGamepad::default());

    tonic::transport::Server::builder()
        .add_service(client.into_service())
        .add_service(speaker.into_service())
        .add_service(base.into_service())
        .add_service(nav.into_service())
        .add_service(loc.into_service())
        .add_service(resolver.into_service())
        .add_service(gamepad.into_service())
        .serve(addr)
        .await?;

    Ok(())
}
