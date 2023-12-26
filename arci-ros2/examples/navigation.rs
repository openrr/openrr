use std::time::Duration;

use arci::*;
use arci_ros2::{Node, NodeOptions};
use clap::Parser;

#[derive(Parser, Debug)]
#[clap(name = env!("CARGO_BIN_NAME"))]
struct Args {
    x: f64,
    y: f64,
    yaw: f64,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let node = Node::new(
        "nav2_node",
        "/arci_ros2",
        NodeOptions::new().enable_rosout(true),
    )?;
    // let nav = arci_ros2::navigation_pubsub::Ros2Navigation::new(node, "/goal_pose", "/amcl_pose");
    let nav = arci_ros2::Ros2Navigation::new(node, "/navigate_to_pose");
    // TODO: wait for server
    tokio::time::sleep(Duration::from_secs(1)).await;
    nav.send_goal_pose(
        Isometry2::new(Vector2::new(args.x, args.y), args.yaw),
        "map",
        Duration::from_secs(30),
    )?
    .await?;
    Ok(())
}
