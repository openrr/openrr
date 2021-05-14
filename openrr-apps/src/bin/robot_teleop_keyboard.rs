#![cfg_attr(windows, allow(dead_code, unused_imports))]

#[cfg(feature = "ros")]
use std::thread;
use std::{path::PathBuf, sync::Arc};

#[cfg(unix)]
use arci_gamepad_keyboard::KeyboardGamepad;
use openrr_client::ArcRobotClient;
use openrr_teleop::ControlNodeSwitcher;
use structopt::StructOpt;
use tracing::info;

#[derive(StructOpt, Debug)]
#[structopt(
    name = env!("CARGO_BIN_NAME"),
    about = "An openrr teleoperation tool.",
)]
struct Args {
    /// Path to the setting file.
    #[structopt(short, long, parse(from_os_str))]
    config_path: Option<PathBuf>,
    /// Prints the default setting as TOML.
    #[structopt(long)]
    show_default_config: bool,
}
use openrr_apps::{Error, RobotConfig, RobotTeleopConfig};

#[cfg(windows)]
fn main() {
    println!("{} is not supported on windows", env!("CARGO_BIN_NAME"));
}

#[cfg(unix)]
#[tokio::main]
async fn main() -> Result<(), Error> {
    tracing_subscriber::fmt::init();
    let args = Args::from_args();

    if args.show_default_config {
        print!(
            "{}",
            toml::to_string(&RobotTeleopConfig::default()).unwrap()
        );
        return Ok(());
    }

    let config_path = args.config_path.ok_or(Error::NoConfigPath)?;
    let teleop_config = RobotTeleopConfig::try_new(config_path)?;
    let robot_config =
        RobotConfig::try_new(teleop_config.robot_config_full_path().as_ref().unwrap())?;
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
        return Err(Error::NoSpecifiedNode(teleop_config.initial_mode));
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

    switcher.main(KeyboardGamepad::new()).await;

    Ok(())
}

#[cfg(unix)]
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_args() {
        let bin = env!("CARGO_BIN_NAME");
        assert!(Args::from_iter_safe(&[bin]).is_ok());
        assert!(Args::from_iter_safe(&[bin, "--show-default-config"]).is_ok());
        assert!(Args::from_iter_safe(&[bin, "--config-path", "path"]).is_ok());
        assert!(
            Args::from_iter_safe(&[bin, "--show-default-config", "--config-path", "path"]).is_ok()
        );
    }
}
