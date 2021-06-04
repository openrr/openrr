use std::time::Duration;

use anyhow::Result;
use arci::{BaseVelocity, JointTrajectoryClient, MoveBase, Speaker};
use openrr_plugin::PluginProxy;

#[tokio::main]
async fn main() -> Result<()> {
    let plugin = PluginProxy::from_path("./target/debug/libopenrr_plugin_example.dylib")?;

    println!("Plugin: {}", plugin.name());
    if let Some(joint_trajectory_client) =
        plugin.new_joint_trajectory_client(r#"{ "joint_names": ["a", "b"] }"#.into())?
    {
        println!("joint_names: {:?}", joint_trajectory_client.joint_names());
        println!(
            "current_joint_positions: {:?}",
            joint_trajectory_client.current_joint_positions()?
        );
        println!("sending joint positions: [1.0, -1.0]");
        joint_trajectory_client
            .send_joint_positions(vec![1.0, -1.0], Duration::from_secs(1))?
            .await?;
        println!(
            "current_joint_positions: {:?}",
            joint_trajectory_client.current_joint_positions()?
        );
    }
    if let Some(speaker) = plugin.new_speaker("".into())? {
        speaker.speak("hi!")?.await?;
    }
    if let Some(move_base) = plugin.new_move_base("".into())? {
        println!("current_velocity: {:?}", move_base.current_velocity()?);
        let new = BaseVelocity {
            x: 2.0,
            y: 1.0,
            theta: 0.5,
        };
        println!("sending velocity: {:?}", new);
        move_base.send_velocity(&new)?;
        println!("current_velocity: {:?}", move_base.current_velocity()?);
    }

    Ok(())
}
