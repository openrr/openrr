#[cfg(feature = "ros2")]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use std::time::Duration;

    use arci::*;
    use arci_ros2::{Node, Ros2LocalizationClient};

    let node = Node::new("localization_rx", "arci_ros2")?;
    node.run_spin_thread(Duration::from_millis(100));
    let localization =
        Ros2LocalizationClient::new(node, false, "nomotion_update_service", "/amcl_pose");
    let pose = localization.current_pose("")?;

    println!("pose: {:?}", pose);

    Ok(())
}

#[cfg(not(feature = "ros2"))]
fn main() {
    println!("This example requires ros2 feature");
}
