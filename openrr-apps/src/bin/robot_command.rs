use std::path::PathBuf;

use anyhow::Result;
use clap::{CommandFactory, Parser};
use clap_complete::Shell;
use openrr_apps::{
    utils::{init_tracing, init_tracing_with_file_appender, LogConfig},
    Error, RobotConfig,
};
use openrr_client::BoxRobotClient;
use openrr_command::{RobotCommand, RobotCommandExecutor};
use tracing::info;

/// An openrr command line tool.
#[derive(Parser, Debug)]
#[clap(name = env!("CARGO_BIN_NAME"))]
struct RobotCommandArgs {
    /// Path to the setting file.
    #[clap(short, long, parse(from_os_str))]
    config_path: Option<PathBuf>,
    /// Set options from command line. These settings take priority over the
    /// setting file specified by --config-path.
    #[clap(long)]
    config: Option<String>,
    #[clap(subcommand)]
    command: Option<RobotCommand>,
    /// Prints the default setting as TOML.
    #[clap(long)]
    show_default_config: bool,
    /// Use interactive mode
    #[clap(short, long)]
    interactive: bool,
    /// Path to log directory for tracing FileAppender.
    #[clap(long, parse(from_os_str))]
    log_directory: Option<PathBuf>,
}

fn shell_completion(shell_type: openrr_command::ShellType) {
    clap_complete::generate(
        Shell::from(shell_type),
        &mut RobotCommandArgs::command(),
        env!("CARGO_BIN_NAME"),
        &mut std::io::stdout(),
    );
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = RobotCommandArgs::parse();
    if args.log_directory.is_none() {
        init_tracing();
    }
    #[cfg(not(feature = "ros"))]
    let _guard = args.log_directory.as_ref().map(|log_directory| {
        init_tracing_with_file_appender(
            LogConfig {
                directory: log_directory.to_path_buf(),
                ..Default::default()
            },
            env!("CARGO_BIN_NAME").to_string(),
        )
    });

    info!("ParsedArgs {args:?}");

    if args.show_default_config {
        print!("{}", toml::to_string(&RobotConfig::default()).unwrap());
        return Ok(());
    }

    // Outputs shell completion script and exit.
    if let Some(RobotCommand::ShellCompletion(shell_type)) = args.command.as_ref() {
        shell_completion(*shell_type);
        return Ok(());
    }

    let config_path = openrr_apps::utils::get_apps_robot_config(args.config_path);

    let robot_config =
        openrr_apps::utils::resolve_robot_config(config_path.as_deref(), args.config.as_deref())?;

    openrr_apps::utils::init_with_anonymize(env!("CARGO_BIN_NAME"), &robot_config);
    #[cfg(feature = "ros")]
    let _guard = args.log_directory.map(|log_directory| {
        init_tracing_with_file_appender(
            LogConfig {
                directory: log_directory,
                ..Default::default()
            },
            if robot_config.has_ros_clients() {
                arci_ros::name()
            } else {
                env!("CARGO_BIN_NAME").to_string()
            },
        )
    });
    let client: BoxRobotClient = robot_config.create_robot_client()?;
    let executor = RobotCommandExecutor {};

    if args.interactive {
        Ok(executor.run_interactive_shell(&client).await?)
    } else {
        let command = args.command.ok_or(Error::NoCommand)?;
        Ok(executor.execute(&client, &command).await?)
    }
}

#[cfg(test)]
mod tests {
    use std::{env, path::Path};

    use openrr_command::load_command_file_and_filter;
    use tracing::log::error;

    use super::*;

    #[test]
    fn parse_args() {
        let bin = env!("CARGO_BIN_NAME");
        assert!(RobotCommandArgs::try_parse_from(&[bin]).is_ok());
        assert!(RobotCommandArgs::try_parse_from(&[bin, "--show-default-config"]).is_ok());
        assert!(RobotCommandArgs::try_parse_from(&[bin, "--config-path", "path", "list"]).is_ok());
        assert!(RobotCommandArgs::try_parse_from(&[
            bin,
            "--show-default-config",
            "--config-path",
            "path"
        ])
        .is_ok());
    }

    #[test]
    fn assert_app() {
        RobotCommandArgs::command().debug_assert();
    }

    const COMMAND_PATHS: [&str; 11] = [
        "command/sample_cmd_urdf_viz.txt",
        "command/pr2_cmd_collision.txt",
        "command/pr2_cmd_ik.txt",
        "command/pr2_cmd_joints.txt",
        "command/pr2_cmd_ros.txt",
        "command/pr2_cmd_urdf_viz.txt",
        "command/ur10_cmd_collision.txt",
        "command/ur10_cmd_ik.txt",
        "command/ur10_cmd_joints.txt",
        "command/ur10_cmd_ros.txt",
        "command/ur10_cmd_urdf_viz.txt",
    ];

    #[test]
    fn assert_commands() {
        for command_path in COMMAND_PATHS {
            let bin = env!("CARGO_BIN_NAME");
            let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));

            let config_path = manifest_dir.join("config/sample_robot_client_for_urdf_viz.toml");
            let config_path_arg = "--config-path=".to_string() + config_path.to_str().unwrap();
            let sample_command_path = manifest_dir.join(command_path);

            let sample_command = RobotCommandArgs::try_parse_from(&[
                bin,
                config_path_arg.as_str(),
                "load_commands",
                sample_command_path.to_str().unwrap(),
            ])
            .unwrap();
            let mut results = Vec::new();

            let command = sample_command.command.ok_or(Error::NoCommand).unwrap();

            check_command(&command, &mut results, command_path);

            for result in results {
                assert!(result);
            }
        }
    }

    fn check_command(command: &RobotCommand, log: &mut Vec<bool>, command_path: &str) {
        match &command {
            RobotCommand::SendJoints {
                name: _,
                duration,
                use_interpolation: _,
                joint: _,
                max_resolution_for_interpolation: _,
                min_number_of_points_for_interpolation: _,
            } => {
                log.push(*duration >= 0f64);
            }
            RobotCommand::SendJointsPose {
                name: _,
                pose_name: _,
                duration,
            } => {
                log.push(*duration >= 0f64);
            }
            RobotCommand::MoveIk {
                name: _,
                x: _,
                y: _,
                z: _,
                yaw: _,
                pitch: _,
                roll: _,
                duration,
                use_interpolation: _,
                is_local: _,
                max_resolution_for_interpolation: _,
                min_number_of_points_for_interpolation: _,
            } => {
                log.push(*duration >= 0f64);
            }
            RobotCommand::GetState { name: _ } => log.push(true),
            RobotCommand::LoadCommands { command_file_path } => {
                let target_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
                let target_command_file_path = target_dir.join(command_path);
                log.push(*command_file_path == target_command_file_path);

                for command in load_command_file_and_filter(target_command_file_path).unwrap() {
                    let command_parsed_iter = command.split_whitespace();
                    let read_opt = RobotCommand::try_parse_from(command_parsed_iter);
                    match read_opt {
                        Ok(robot_command) => {
                            check_command(&robot_command, log, command_path);
                        }
                        Err(err) => {
                            error!("Error in {:?}: {}", command_file_path, err);
                            log.push(false);
                        }
                    }
                }
            }
            RobotCommand::List => {
                log.push(true);
            }
            RobotCommand::Speak {
                name: _,
                message: _,
            } => {
                log.push(true);
            }
            RobotCommand::ExecuteCommand { command: _ } => {
                log.push(true);
            }
            RobotCommand::GetNavigationCurrentPose => {
                log.push(true);
            }
            RobotCommand::SendNavigationGoal {
                x: _,
                y: _,
                yaw: _,
                frame_id: _,
                timeout_secs: _,
            } => {
                log.push(true);
            }
            RobotCommand::CancelNavigationGoal => {
                log.push(true);
            }
            RobotCommand::SendBaseVelocity {
                x: _,
                y: _,
                theta: _,
                duration_secs,
            } => {
                log.push(*duration_secs >= 0f64);
            }
            _ => {
                log.push(false);
            }
        }
    }
}
