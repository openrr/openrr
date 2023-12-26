use arci::Localization;
use arci_ros2::{Node, NodeOptions, Ros2LocalizationClient};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let node = Node::new(
        "ros2_localization_client",
        "/arci_ros2",
        NodeOptions::new().enable_rosout(true),
    )?;
    let c = Ros2LocalizationClient::new(node, false, "/request_nomotion_update", "/amcl_pose")?;
    println!("{:?}", c.current_pose("")?);
    Ok(())
}
