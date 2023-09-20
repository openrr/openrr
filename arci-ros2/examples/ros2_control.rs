#[cfg(feature = "ros2")]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use std::time::Duration;

    use arci::*;
    use arci_ros2::{Node, Ros2ControlClient};

    let node = Node::new("ros2_control_node", "arci_ros2").unwrap();
    node.run_spin_thread(Duration::from_millis(100));
    let client = Ros2ControlClient::new(node, "/position_trajectory_controller");
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

#[cfg(not(feature = "ros2"))]
fn main() {
    println!("This example requires ros2 feature");
}
