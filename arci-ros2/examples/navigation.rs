#[cfg(feature = "ros2")]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use std::time::Duration;

    use arci::*;
    use arci_ros2::{r2r, Ros2Navigation};

    let ctx = r2r::Context::create().unwrap();
    let nav = Ros2Navigation::new(ctx, "/navigate_to_pose");
    nav.send_goal_pose(
        Isometry2::new(Vector2::new(-0.6, 0.2), 1.0),
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
