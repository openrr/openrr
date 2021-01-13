use log::info;
use openrr_apps::{Error, RobotCommand};
use structopt::StructOpt;

#[tokio::main]
async fn main() -> Result<(), Error> {
    env_logger::init();
    let command = RobotCommand::from_args();
    info!("ParsedCommand {:?}", command);
    #[cfg(feature = "ros")]
    arci_ros::init("openrr_apps_robot_command", true);
    Ok(command.execute().await?)
}
