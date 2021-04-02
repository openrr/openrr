use arci::JointTrajectoryClient;
use arci_urdf_viz::UrdfVizWebClient;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = UrdfVizWebClient::default();
    let pos = client.current_joint_positions()?;
    let dof = pos.len();
    client
        .send_joint_positions(&vec![0.0; dof], std::time::Duration::from_millis(1000))?
        .wait()?;
    Ok(())
}
