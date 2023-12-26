use std::time::Duration;

use arci::*;
use arci_ros2::{Node, NodeOptions, Ros2ControlClient};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let node = Node::new(
        "ros2_control_node",
        "/arci_ros2",
        NodeOptions::new().enable_rosout(true),
    )?;
    let client = Ros2ControlClient::new(node, "/joint_trajectory_position_controller")?;
    dbg!(client.joint_names()); // => ["joint1", "joint2"]
    dbg!(client.current_joint_positions()).unwrap();
    client
        .send_joint_positions(
            vec![1.0; client.joint_names().len()],
            Duration::from_secs(5),
        )?
        .await?;
    dbg!(client.current_joint_positions()).unwrap();
    client
        .send_joint_positions(
            vec![0.5; client.joint_names().len()],
            Duration::from_secs(5),
        )?
        .await?;
    dbg!(client.current_joint_positions()).unwrap();

    Ok(())
}
