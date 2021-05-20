#[cfg(feature = "ros")]
use std::thread;
use std::{fs, path::PathBuf, sync::Arc};

use anyhow::Result;
use arci_gamepad_gilrs::GilGamepad;
use openrr_apps::{Error, GamepadKind, RobotConfig, RobotTeleopConfig};
use openrr_client::ArcRobotClient;
use openrr_teleop::ControlNodeSwitcher;
use structopt::StructOpt;
use tracing::info;

/// An openrr teleoperation tool.
#[derive(StructOpt, Debug)]
#[structopt(name = env!("CARGO_BIN_NAME"))]
pub struct RobotTeleopArgs {
    /// Path to the setting file.
    #[structopt(short, long, parse(from_os_str))]
    config_path: Option<PathBuf>,
    /// Set options from command line. These settings take priority over the
    /// setting file specified by --config-path.
    #[structopt(long)]
    teleop_config: Option<String>,
    /// Set options from command line. These settings take priority over the
    /// setting file specified by --config-path.
    #[structopt(long)]
    robot_config: Option<String>,
    /// Prints the default setting as TOML.
    #[structopt(long)]
    show_default_config: bool,
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    let args = RobotTeleopArgs::from_args();

    if args.show_default_config {
        print!(
            "{}",
            toml::to_string(&RobotTeleopConfig::default()).unwrap()
        );
        return Ok(());
    }

    let teleop_config_path = args.config_path.ok_or(Error::NoConfigPath)?;
    let teleop_config = if let Some(overwrite) = &args.teleop_config {
        let s = &fs::read_to_string(&teleop_config_path)?;
        // check if the input is valid config.
        let _base: RobotTeleopConfig = toml::from_str(s)?;
        let mut edit: toml::Value = toml::from_str(s)?;
        openrr_config::overwrite(&mut edit, overwrite)?;
        RobotTeleopConfig::from_str(&toml::to_string(&edit)?, &teleop_config_path)?
    } else {
        RobotTeleopConfig::try_new(teleop_config_path)?
    };
    let robot_config_path = teleop_config.robot_config_full_path().as_ref().unwrap();
    let robot_config = if let Some(overwrite) = &args.robot_config {
        let s = &fs::read_to_string(&robot_config_path)?;
        // check if the input is valid config.
        let _base: RobotConfig = toml::from_str(s)?;
        let mut edit: toml::Value = toml::from_str(s)?;
        openrr_config::overwrite(&mut edit, overwrite)?;
        RobotConfig::from_str(&toml::to_string(&edit)?, robot_config_path)?
    } else {
        RobotConfig::try_new(robot_config_path)?
    };

    openrr_apps::utils::init(env!("CARGO_BIN_NAME"), &robot_config);
    #[cfg(feature = "ros")]
    let use_ros = robot_config.has_ros_clients();
    let client: Arc<ArcRobotClient> = Arc::new(robot_config.create_robot_client()?);

    let speaker = client.speakers().values().next().unwrap();

    let nodes = teleop_config.control_nodes_config.create_control_nodes(
        speaker.clone(),
        client.joint_trajectory_clients(),
        client.ik_solvers(),
        Some(client.clone()),
        robot_config.openrr_clients_config.joints_poses,
    );
    if nodes.is_empty() {
        panic!("No valid nodes");
    }

    let initial_node_index = if teleop_config.initial_mode.is_empty() {
        info!("Use first node as initial node");
        0
    } else if let Some(initial_node_index) = nodes
        .iter()
        .position(|node| node.mode() == teleop_config.initial_mode)
    {
        initial_node_index
    } else {
        return Err(Error::NoSpecifiedNode(teleop_config.initial_mode).into());
    };

    let switcher = Arc::new(ControlNodeSwitcher::new(
        nodes,
        speaker.clone(),
        initial_node_index,
    ));
    #[cfg(feature = "ros")]
    if use_ros {
        let switcher_cloned = switcher.clone();
        thread::spawn(move || {
            let rate = arci_ros::rate(1.0);
            while arci_ros::is_ok() {
                rate.sleep();
            }
            switcher_cloned.stop();
        });
    }

    match teleop_config.gamepad {
        GamepadKind::Gilrs => {
            switcher
                .main(GilGamepad::new_from_config(
                    teleop_config.gil_gamepad_config,
                ))
                .await;
        }
        #[cfg(unix)]
        GamepadKind::Keyboard => {
            switcher
                .main(arci_gamepad_keyboard::KeyboardGamepad::new())
                .await;
        }
        #[cfg(windows)]
        GamepadKind::Keyboard => {
            tracing::warn!("`gamepad = \"Keyboard\"` is not supported on windows");
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_args() {
        let bin = env!("CARGO_BIN_NAME");
        assert!(RobotTeleopArgs::from_iter_safe(&[bin]).is_ok());
        assert!(RobotTeleopArgs::from_iter_safe(&[bin, "--show-default-config"]).is_ok());
        assert!(RobotTeleopArgs::from_iter_safe(&[bin, "--config-path", "path"]).is_ok());
        assert!(RobotTeleopArgs::from_iter_safe(&[
            bin,
            "--show-default-config",
            "--config-path",
            "path"
        ])
        .is_ok());
    }
}
