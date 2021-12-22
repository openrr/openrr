#[cfg(feature = "ros2")]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use std::time::Duration;

    use arci::*;
    use arci_ros2::{r2r, Ros2ControlClient};

    let ctx = r2r::Context::create().unwrap();
    let client = Ros2ControlClient::new(
        ctx,
        "/position_trajectory_controller/follow_joint_trajectory",
        vec!["joint1".into(), "joint2".into()],
    );
    client
        .send_joint_positions(vec![1.0, 1.0], Duration::from_secs(10))?
        .await?;
    Ok(())
}

#[cfg(not(feature = "ros2"))]
fn main() {
    println!("This example requires ros2 feature");
}
