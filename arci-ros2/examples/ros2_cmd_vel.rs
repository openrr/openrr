use arci::{BaseVelocity, MoveBase};
use arci_ros2::{Node, NodeOptions, Ros2CmdVelMoveBase};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let node = Node::new(
        "cmd_vel_node",
        "/arci_ros2",
        NodeOptions::new().enable_rosout(true),
    )?;
    let c = Ros2CmdVelMoveBase::new(node, "/cmd_vel");
    let mut count = 0;
    let mut vel = BaseVelocity::default();
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
