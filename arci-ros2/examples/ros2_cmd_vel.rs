#[cfg(feature = "ros2")]
#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use arci::{BaseVelocity, MoveBase};
    use arci_ros2::{r2r, Ros2CmdVelMoveBase};

    let ctx = r2r::Context::create().unwrap();
    let c = Ros2CmdVelMoveBase::new(ctx, "/cmd_vel");
    let mut count = 0;
    let mut vel = BaseVelocity::default();
    while count < 100 {
        vel.x = 0.001 * (count as f64);
        c.send_velocity(&vel)?;
        std::thread::sleep(std::time::Duration::from_millis(100));
        count += 1;
        println!("{}, {:?}", count, vel);
    }
    while count >= 0 {
        vel.x = 0.001 * (count as f64);
        c.send_velocity(&vel)?;
        std::thread::sleep(std::time::Duration::from_millis(100));
        count -= 1;
        println!("{}, {:?}", count, vel);
    }
    Ok(())
}

#[cfg(not(feature = "ros2"))]
fn main() {
    println!("This example requires ros2 feature");
}
