use openrr_apps::RobotConfig;
use openrr_client::BoxRobotClient;
use openrr_gui::joint_position_sender;
use std::path::PathBuf;
use structopt::StructOpt;
use tracing::debug;

/// An openrr GUI tool.
#[derive(StructOpt, Debug)]
#[structopt(name = env!("CARGO_BIN_NAME"))]
struct Opt {
    /// Path to the setting file.
    #[structopt(short, long, parse(from_os_str))]
    config_path: Option<PathBuf>,
}

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();
    let opt = Opt::from_args();
    debug!("opt: {:?}", opt);
    let config_path = openrr_apps::utils::get_apps_robot_config(opt.config_path).unwrap();
    let config = RobotConfig::try_new(&config_path)?;
    openrr_apps::utils::init(env!("CARGO_BIN_NAME"), &config);
    let client: BoxRobotClient = config.create_robot_client()?;
    joint_position_sender(
        client,
        config.openrr_clients_config.urdf_full_path().unwrap(),
    )?;
    Ok(())
}
