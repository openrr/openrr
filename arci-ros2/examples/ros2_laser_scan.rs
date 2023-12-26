use arci::LaserScan2D;
use arci_ros2::{Node, NodeOptions, Ros2LaserScan2D};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let node = Node::new(
        "ros2_laser_scan",
        "/arci_ros2",
        NodeOptions::new().enable_rosout(true),
    )?;
    let c = Ros2LaserScan2D::new(node, "/scan")?;
    println!("{:?}", c.current_scan()?);
    Ok(())
}
