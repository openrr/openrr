use anyhow::Error;
use arci::JointTrajectoryClient;
use arci_ros::RosRobotClient;

#[tokio::main]
async fn main() -> Result<(), Error> {
    arci_ros::init("example");
    let chain = k::Chain::<f64>::from_urdf_file("../dobot_description/dobot_without_hand.urdf")?;
    let c = RosRobotClient::new(
        chain.iter_joints().map(|j| j.name.to_string()).collect(),
        "joint_states",
        "joint_trajectory",
    );
    let rate = arci_ros::rate(1.0);
    let angles = vec![1.0, 2.0, 3.0, 4.0];
    while arci_ros::is_ok() {
        c.send_joint_positions(angles.clone(), std::time::Duration::from_secs(1))
            .await?;
        println!("{:?}", c.current_joint_positions());
        rate.sleep();
    }
    Ok(())
}
