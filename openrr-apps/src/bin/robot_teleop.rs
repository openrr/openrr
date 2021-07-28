#[cfg(feature = "ros")]
use std::thread;
use std::{fs, path::PathBuf, sync::Arc};

use anyhow::{format_err, Result};
use arci_gamepad_gilrs::GilGamepad;
use openrr_apps::{
    utils::init_tracing, BuiltinGamepad, Error, GamepadKind, RobotConfig, RobotTeleopConfig,
};
use openrr_client::ArcRobotClient;
use openrr_plugin::PluginProxy;
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
    init_tracing();
    let args = RobotTeleopArgs::from_args();

    if args.show_default_config {
        print!(
            "{}",
            toml::to_string(&RobotTeleopConfig::default()).unwrap()
        );
        return Ok(());
    }

    let teleop_config = match (&args.config_path, &args.teleop_config) {
        (Some(teleop_config_path), Some(overwrite)) => {
            let s = &fs::read_to_string(&teleop_config_path)?;
            let s = &openrr_config::overwrite_str(s, overwrite)?;
            RobotTeleopConfig::from_str(s, teleop_config_path)?
        }
        (Some(teleop_config_path), None) => RobotTeleopConfig::new(teleop_config_path)?,
        (None, overwrite) => {
            let mut config = RobotTeleopConfig::default();
            config.control_nodes_config.move_base_mode = Some("base".into());
            if let Some(overwrite) = overwrite {
                let s = &toml::to_string(&config)?;
                let s = &openrr_config::overwrite_str(s, overwrite)?;
                config = toml::from_str(s)?;
            }
            config
        }
    };
    let robot_config_path = teleop_config.robot_config_full_path();
    let robot_config = match (robot_config_path, &args.robot_config) {
        (Some(config_path), Some(overwrite)) => {
            let s = &fs::read_to_string(&config_path)?;
            let s = &openrr_config::overwrite_str(s, overwrite)?;
            RobotConfig::from_str(s, config_path)?
        }
        (Some(config_path), None) => RobotConfig::new(config_path)?,
        (None, overwrite) => {
            let mut config = RobotConfig::default();
            config
                .urdf_viz_clients_configs
                .push(arci_urdf_viz::UrdfVizWebClientConfig {
                    name: "all".into(),
                    joint_names: None,
                    wrap_with_joint_position_limiter: false,
                    wrap_with_joint_velocity_limiter: false,
                    joint_velocity_limits: None,
                    joint_position_limits: None,
                });
            if let Some(overwrite) = overwrite {
                let s = &toml::to_string(&config)?;
                let s = &openrr_config::overwrite_str(s, overwrite)?;
                config = toml::from_str(s)?;
            }
            config
        }
    };

    openrr_apps::utils::init(env!("CARGO_BIN_NAME"), &robot_config);
    #[cfg(feature = "ros")]
    let use_ros = robot_config.has_ros_clients();
    let client: Arc<ArcRobotClient> = Arc::new(robot_config.create_robot_client()?);

    let speaker = client.speakers().values().next().unwrap();

    let nodes = teleop_config.control_nodes_config.create_control_nodes(
        args.config_path,
        client.clone(),
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
        GamepadKind::Builtin(BuiltinGamepad::Gilrs) => {
            switcher
                .main(GilGamepad::new_from_config(
                    teleop_config.gil_gamepad_config,
                ))
                .await;
        }
        #[cfg(unix)]
        GamepadKind::Builtin(BuiltinGamepad::Keyboard) => {
            switcher
                .main(arci_gamepad_keyboard::KeyboardGamepad::new())
                .await;
        }
        #[cfg(windows)]
        GamepadKind::Builtin(BuiltinGamepad::Keyboard) => {
            tracing::warn!("`gamepad = \"Keyboard\"` is not supported on windows");
        }
        GamepadKind::Plugin(name) => {
            let mut gamepad = None;
            for (plugin_name, config) in teleop_config.plugins {
                if name == plugin_name {
                    let args = if let Some(path) = &config.args_from_path {
                        fs::read_to_string(path).map_err(|e| Error::NoFile(path.to_owned(), e))?
                    } else {
                        config.args.unwrap_or_default()
                    };
                    let plugin = PluginProxy::from_path(&config.path)?;
                    gamepad = Some(plugin.new_gamepad(args)?.ok_or_else(|| {
                        format_err!("failed to create `Gamepad` instance `{}`: None", name,)
                    })?);
                    break;
                }
            }
            match gamepad {
                Some(gamepad) => {
                    switcher.main(gamepad).await;
                }
                None => {
                    return Err(Error::NoPluginInstance {
                        name,
                        kind: "Gamepad".to_string(),
                    }
                    .into());
                }
            }
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
