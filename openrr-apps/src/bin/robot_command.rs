use crate::Error as OpenrrAppsError;
use log::info;
use openrr_apps::{Error, RobotConfig};
use openrr_command::{RobotCommand, RobotCommandExecutor};
use std::path::PathBuf;
use structopt::StructOpt;

#[derive(StructOpt, Debug)]
#[structopt(
    name = env!("CARGO_BIN_NAME"),
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
    if let Some(config_path) = openrr_apps::utils::get_apps_robot_config(args.config_path) {
        let robot_config = RobotConfig::try_new(config_path)?;
        openrr_apps::utils::init_with_anonymize(env!("CARGO_BIN_NAME"), &robot_config);
        let client = robot_config.create_robot_client()?;
        let executor = RobotCommandExecutor {};
        Ok(executor.execute(&client, &args.command).await?)
    } else {
        Err(OpenrrAppsError::NoConfigPath)
    }
}
