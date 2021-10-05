#[cfg(feature = "ros2")]
#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use arci::{Isometry2, Navigation, Vector2};
    use arci_ros2::{r2r, Ros2Navigation};

    let ctx = r2r::Context::create().unwrap();
    let n = Ros2Navigation::new(ctx, "/navigate_to_pose");
    n.send_goal_pose(
        Isometry2::new(Vector2::new(0.6, 0.2), 1.0),
        "map",
        std::time::Duration::from_secs_f64(10.0),
    )?
    .await?;
    Ok(())
}

#[cfg(not(feature = "ros2"))]
fn main() {}
