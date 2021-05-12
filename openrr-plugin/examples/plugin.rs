use std::time::Duration;

use anyhow::Result;
use arci::{BaseVelocity, JointTrajectoryClient, MoveBase, Speaker};
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
                joint_trajectory_client.current_joint_positions()?
            );
            println!("setting joint positions to [1.0, -1.0]");
            joint_trajectory_client
                .send_joint_positions(vec![1.0, -1.0], Duration::from_secs(1))?
                .await?;
            println!(
                "current_joint_positions: {:?}",
                joint_trajectory_client.current_joint_positions()?
            );
        }
        if let Some(speaker) = plugin.speaker() {
            speaker.speak("hi!")?.await?;
        }
        if let Some(move_base) = plugin.move_base() {
            println!("current_velocity: {:?}", move_base.current_velocity()?);
            let new = BaseVelocity {
                x: 2.0,
                y: 1.0,
                theta: 0.5,
            };
            println!("setting velocity {:?}", new);
            move_base.send_velocity(&new)?;
            println!("current_velocity: {:?}", move_base.current_velocity()?);
        }
    }

    Ok(())
}
