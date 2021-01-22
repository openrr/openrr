use crate::Error as OpenrrAppsError;
use log::info;
use openrr_apps::{Error, RobotCommand, RobotCommandExecutor, RobotConfig};
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
    #[cfg(feature = "ros")]
    {
        let suffix: u64 = rand::thread_rng().gen();
        arci_ros::init(&format!("openrr_apps_robot_command_{}", suffix));
    }
    if let Some(config_path) = &args.config_path {
        let client = RobotConfig::try_new(config_path)?.create_robot_client()?;
        let executor = RobotCommandExecutor {};
        Ok(executor.execute(&client, &args.command).await?)
    } else {
        return Err(OpenrrAppsError::NoConfigPath);
    }
}
