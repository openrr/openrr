// TODO: move to openrr-apps?

use log::debug;
use openrr_apps::RobotConfig;
use openrr_client::BoxRobotClient;
use openrr_gui::joint_position_sender;
use structopt::StructOpt;

#[derive(StructOpt, Debug)]
struct Opt {
    config_path: String,
    urdf_path: String,
}

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let opt = Opt::from_args();
    debug!("opt: {:?}", opt);
    let config = RobotConfig::try_new(&opt.config_path)?;
    let client: BoxRobotClient = config.create_robot_client()?;

    joint_position_sender(client, opt.urdf_path)?;
    Ok(())
}
