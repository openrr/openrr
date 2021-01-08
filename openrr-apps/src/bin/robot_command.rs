use log::info;
use openrr_apps::{Error, RobotCommand};
#[cfg(feature = "ros")]
use rand::prelude::*;
use structopt::StructOpt;

#[tokio::main]
async fn main() -> Result<(), Error> {
    env_logger::init();
    let command = RobotCommand::from_args();
    info!("ParsedCommand {:?}", command);
    #[cfg(feature = "ros")]
    {
        let suffix: u64 = rand::thread_rng().gen();
        arci_ros::init(&format!("openrr_apps_robot_command_{}", suffix));
    }
    Ok(command.execute().await?)
}
