use std::time::Duration;

use anyhow::Result;
use arci::JointTrajectoryClient;
use openrr_plugin::PluginManager;

#[tokio::main]
async fn main() -> Result<()> {
    let mut pm = PluginManager::new();
    pm.load("./target/debug/libopenrr_plugin_example.dylib")?;

    println!("Collected {} plugin(s)", pm.plugins().len());
    for plugin in pm.plugins() {
        println!("Plugin: {}", plugin.name());
        if let Some(joint_trajectory_client) = plugin.joint_trajectory_client() {
            println!("joint_names: {:?}", joint_trajectory_client.joint_names());
            println!(
                "current_joint_positions: {:?}",
                joint_trajectory_client.current_joint_positions()
            );
            println!("setting joint positions to [1.0, -1.0]");
            joint_trajectory_client
                .send_joint_positions(vec![1.0, -1.0], Duration::from_secs(1))?
                .await?;
            println!(
                "current_joint_positions: {:?}",
                joint_trajectory_client.current_joint_positions()
            );
        }
    }

    Ok(())
}
