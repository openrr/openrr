#[cfg(feature = "ros")]
use std::thread;
use std::{fs, path::PathBuf, sync::Arc};

use anyhow::{format_err, Result};
use arci_gamepad_gilrs::GilGamepad;
use clap::Parser;
use openrr_apps::{
    utils::{init_tracing, init_tracing_with_file_appender, LogConfig},
    BuiltinGamepad, Error, GamepadKind, RobotTeleopConfig,
};
use openrr_client::ArcRobotClient;
use openrr_plugin::PluginProxy;
use openrr_teleop::ControlModeSwitcher;
use tracing::info;

/// An openrr teleoperation tool.
#[derive(Parser, Debug)]
#[clap(name = env!("CARGO_BIN_NAME"))]
pub struct RobotTeleopArgs {
    /// Path to the setting file.
    #[clap(short, long, value_parser)]
    config_path: Option<PathBuf>,
    /// Set options from command line. These settings take priority over the
    /// setting file specified by --config-path.
    #[clap(long)]
    teleop_config: Option<String>,
    /// Set options from command line. These settings take priority over the
    /// setting file specified by --config-path.
    #[clap(long)]
    robot_config: Option<String>,
    /// Prints the default setting as TOML.
    #[clap(long)]
    show_default_config: bool,
    /// Path to log directory for tracing FileAppender.
    #[clap(long, value_parser)]
    log_directory: Option<PathBuf>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = RobotTeleopArgs::parse();

    if args.show_default_config {
        print!(
            "{}",
            toml::to_string(&RobotTeleopConfig::default()).unwrap()
        );
        return Ok(());
    }

    if args.log_directory.is_none() {
        init_tracing();
    }
    #[cfg(not(feature = "ros"))]
    let _guard = args.log_directory.map(|log_directory| {
        init_tracing_with_file_appender(
            LogConfig {
                directory: log_directory,
                ..Default::default()
            },
            env!("CARGO_BIN_NAME").to_string(),
        )
    });

    let teleop_config = openrr_apps::utils::resolve_teleop_config(
        args.config_path.as_deref(),
        args.teleop_config.as_deref(),
    )?;
    let robot_config_path = teleop_config.robot_config_full_path();
    let robot_config = openrr_apps::utils::resolve_robot_config(
        robot_config_path.as_deref(),
        args.robot_config.as_deref(),
    )?;

    openrr_apps::utils::init(env!("CARGO_BIN_NAME"), &robot_config);
    #[cfg(feature = "ros")]
    let use_ros = robot_config.has_ros_clients();
    #[cfg(feature = "ros")]
    let _guard = args.log_directory.map(|log_directory| {
        init_tracing_with_file_appender(
            LogConfig {
                directory: log_directory,
                ..Default::default()
            },
            if use_ros {
                arci_ros::name()
            } else {
                env!("CARGO_BIN_NAME").to_string()
            },
        )
    });
    let client: Arc<ArcRobotClient> = Arc::new(robot_config.create_robot_client()?);

    let speaker = client.speakers().values().next().unwrap();

    let modes = teleop_config
        .control_modes_config
        .create_control_modes(
            args.config_path,
            client.clone(),
            speaker.clone(),
            client.joint_trajectory_clients(),
            client.ik_solvers(),
            Some(client.clone()),
            robot_config.openrr_clients_config.joints_poses,
        )
        .unwrap();
    if modes.is_empty() {
        panic!("No valid modes");
    }

    let initial_mode_index = if teleop_config.initial_mode.is_empty() {
        info!("Use first mode as initial mode");
        0
    } else if let Some(initial_mode_index) = modes
        .iter()
        .position(|mode| mode.mode() == teleop_config.initial_mode)
    {
        initial_mode_index
    } else {
        return Err(Error::NoSpecifiedMode(teleop_config.initial_mode).into());
    };

    let switcher = Arc::new(ControlModeSwitcher::new(
        modes,
        speaker.clone(),
        initial_mode_index,
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
            anyhow::bail!("`gamepad = \"keyboard\"` is not supported on windows");
        }
        #[cfg(feature = "ros")]
        GamepadKind::Builtin(BuiltinGamepad::RosJoyGamepad) => {
            if !use_ros {
                arci_ros::init(env!("CARGO_BIN_NAME"));
            }
            switcher
                .main(arci_ros::RosJoyGamepad::new_from_config(
                    &teleop_config.ros_joy_gamepad_config,
                ))
                .await;
        }
        #[cfg(not(feature = "ros"))]
        GamepadKind::Builtin(BuiltinGamepad::RosJoyGamepad) => {
            anyhow::bail!("`gamepad = \"ros-joy-gamepad\"` requires \"ros\" feature");
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
                        format_err!("failed to create `Gamepad` instance `{name}`: None")
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
    use clap::CommandFactory;

    use super::*;

    #[test]
    fn parse_args() {
        let bin = env!("CARGO_BIN_NAME");
        assert!(RobotTeleopArgs::try_parse_from([bin]).is_ok());
        assert!(RobotTeleopArgs::try_parse_from([bin, "--show-default-config"]).is_ok());
        assert!(RobotTeleopArgs::try_parse_from([bin, "--config-path", "path"]).is_ok());
        assert!(RobotTeleopArgs::try_parse_from([
            bin,
            "--show-default-config",
            "--config-path",
            "path"
        ])
        .is_ok());
    }

    #[test]
    fn assert_app() {
        RobotTeleopArgs::command().debug_assert();
    }
}
