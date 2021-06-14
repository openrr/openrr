#[cfg(feature = "ros2")]
#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use arci::*;
    use arci_ros2::{r2r, Ros2Navigation};
    use std::sync::{Arc, Mutex};

    let ctx = r2r::Context::create().unwrap();
    let n = Ros2Navigation::new(ctx, "/navigate_to_pose");
    n.send_goal_pose(
        Isometry2::new(Vector2::new(-0.6, 0.2), 1.0),
        "frame_id",
        std::time::Duration::from_secs_f64(10.0),
    )?
    .await?;
    Ok(())
}

#[cfg(not(feature = "ros2"))]
fn main() {}
