use anyhow::Result;
use openrr_client::BoxRobotClient;
use openrr_gui::joint_position_sender;
// use std::path::PathBuf;
// use structopt::StructOpt;
// use tracing::debug;

#[path = "m/robot_config.rs"]
mod robot_config;
use robot_config::RobotConfig;

// /// An openrr GUI tool.
// #[derive(StructOpt, Debug)]
// struct Opt {
//     /// Path to the setting file.
//     #[structopt(short, long, parse(from_os_str))]
//     config_path: Option<PathBuf>,
// }

fn main() -> Result<()> {
    // tracing_subscriber::fmt::init();
    // let opt = Opt::from_args();
    // debug!("opt: {:?}", opt);
    // let config_path = opt.config_path.unwrap();
    let config: RobotConfig = toml::from_str(include_str!(
        "../../openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml"
    ))?;
    let client: BoxRobotClient = config.create_robot_client()?;
    joint_position_sender(
        client,
        urdf_rs::read_from_string(include_str!("../../openrr-planner/sample.urdf"))?,
    )?;
    Ok(())
}
