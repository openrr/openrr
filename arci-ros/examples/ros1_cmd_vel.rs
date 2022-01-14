#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use arci::{BaseVelocity, MoveBase};
    use arci_ros::RosCmdVelMoveBase;

    // Initialize node
    arci_ros::init("arci_ros_cmd_vel_example");
    let c = RosCmdVelMoveBase::new("/cmd_vel");
    let mut count = 0;
    let mut vel = BaseVelocity::default();
    println!("\"cmd_vel\" Publisher is running!");
    while count < 100 {
        vel.x = 0.001 * (count as f64);
        c.send_velocity(&vel)?;
        std::thread::sleep(std::time::Duration::from_millis(100));
        count += 1;
        println!("{count}, {vel:?}");
    }
    while count >= 0 {
        vel.x = 0.001 * (count as f64);
        c.send_velocity(&vel)?;
        std::thread::sleep(std::time::Duration::from_millis(100));
        count -= 1;
        println!("{count}, {vel:?}");
    }
    Ok(())
}
