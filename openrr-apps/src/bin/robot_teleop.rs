use arci_gamepad_gilrs::{GilGamepad, GilGamepadConfig};
use openrr_apps::{ArcRobotClient, Error, RobotConfig};
use openrr_teleop::{ControlNodeSwitcher, ControlNodesConfig};
use serde::{Deserialize, Serialize};
use std::{path::PathBuf, sync::Arc};
use structopt::StructOpt;

#[derive(Debug, Serialize, Deserialize, Clone)]
struct RobotTeleopConfig {
    pub robot_config_path: String,
    robot_config_full_path: Option<PathBuf>,
    pub control_nodes_config: ControlNodesConfig,
    pub gil_gamepad_config: GilGamepadConfig,
}

impl RobotTeleopConfig {
    pub fn try_new<P: AsRef<std::path::Path>>(path: P) -> Result<Self, Error> {
        let mut config: RobotTeleopConfig = toml::from_str(
            &std::fs::read_to_string(&path)
                .map_err(|e| Error::NoFile(path.as_ref().to_owned(), e))?,
        )
        .map_err(|e| Error::TomlParseFailure(path.as_ref().to_owned(), e))?;
        config.robot_config_full_path = Some(
            path.as_ref()
                .parent()
                .ok_or_else(|| Error::NoParentDirectory(path.as_ref().to_owned()))?
                .join(&config.robot_config_path),
        );
        Ok(config)
    }

    pub fn robot_config_full_path(&self) -> &Option<PathBuf> {
        &self.robot_config_full_path
    }
}

#[derive(StructOpt, Debug)]
#[structopt(
    name = "openrr_apps_robot_teleop",
    about = "An openrr teleoperation tool."
)]
pub struct RobotTeleopArgs {
    #[structopt(short, long, parse(from_os_str))]
    config_path: PathBuf,
}

#[tokio::main]
async fn main() -> Result<(), Error> {
    env_logger::init();
    #[cfg(feature = "ros")]
    arci_ros::init("openrr_apps_robot_teleop");

    let args = RobotTeleopArgs::from_args();

    let teleop_config = RobotTeleopConfig::try_new(args.config_path)?;
    let client: Arc<ArcRobotClient> = Arc::new(
        RobotConfig::try_new(teleop_config.robot_config_full_path().as_ref().unwrap())?
            .create_robot_client()?,
    );

    let nodes = teleop_config.control_nodes_config.create_control_nodes(
        client.clone(),
        client.joint_trajectory_clients(),
        client.ik_solvers(),
        Some(client.clone()),
    );
    if nodes.is_empty() {
        panic!("No valid nodes");
    }

    let switcher = ControlNodeSwitcher::new(nodes, client.clone());
    switcher
        .main(GilGamepad::new_from_config(
            teleop_config.gil_gamepad_config,
        ))
        .await;

    Ok(())
}
