use crate::Error as OpenrrAppsError;
use log::info;
use openrr_apps::{Error, RobotConfig};
use openrr_command::{RobotCommand, RobotCommandExecutor};
#[cfg(feature = "ros")]
use rand::prelude::*;
use std::path::PathBuf;
use structopt::StructOpt;

#[derive(StructOpt, Debug)]
#[structopt(
    name = "openrr_apps_robot_command",
    about = "An openrr command line tool."
)]
struct RobotCommandArgs {
    #[structopt(short, long, parse(from_os_str))]
    config_path: Option<PathBuf>,
    #[structopt(subcommand)]
    command: RobotCommand,
}

#[tokio::main]
async fn main() -> Result<(), Error> {
    env_logger::init();
    let args = RobotCommandArgs::from_args();
    info!("ParsedArgs {:?}", args);
    if let Some(config_path) = &args.config_path {
        let robot_config = RobotConfig::try_new(config_path)?;
        #[cfg(feature = "ros")]
        if robot_config.has_ros_clients() {
            let suffix: u64 = rand::thread_rng().gen();
            arci_ros::init(&format!("openrr_apps_robot_command_{}", suffix));
        }
        let client = robot_config.create_robot_client()?;
        let executor = RobotCommandExecutor {};
        Ok(executor.execute(&client, &args.command).await?)
    } else {
        Err(OpenrrAppsError::NoConfigPath)
    }
}
