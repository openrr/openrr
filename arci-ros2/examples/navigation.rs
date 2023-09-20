#[cfg(feature = "ros2")]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use std::time::Duration;

    use arci::*;
    use arci_ros2::{Node, Ros2Navigation};
    use clap::Parser;

    #[derive(Parser, Debug)]
    #[clap(name = env!("CARGO_BIN_NAME"))]
    struct Args {
        x: f64,
        y: f64,
        yaw: f64,
    }

    let args = Args::parse();
    let node = Node::new("nav2_node", "arci_ros2").unwrap();
    node.run_spin_thread(Duration::from_millis(100));
    let nav = Ros2Navigation::new(node.clone(), "/goal_pose", "/amcl_pose");
    nav.send_goal_pose(
        Isometry2::new(Vector2::new(args.x, args.y), args.yaw),
        "map",
        Duration::from_secs_f64(10.0),
    )?
    .await?;
    Ok(())
}

#[cfg(not(feature = "ros2"))]
fn main() {
    println!("This example requires ros2 feature");
}
