use std::{fs, path::PathBuf};

use anyhow::Result;
use openrr_apps::RobotConfig;
use openrr_client::BoxRobotClient;
use openrr_gui::joint_position_sender;
use structopt::StructOpt;
use tracing::debug;

/// An openrr GUI tool.
#[derive(StructOpt, Debug)]
#[structopt(name = env!("CARGO_BIN_NAME"))]
struct Opt {
    /// Path to the setting file.
    #[structopt(short, long, parse(from_os_str))]
    config_path: Option<PathBuf>,
    /// Set options from command line. These settings take priority over the
    /// setting file specified by --config-path.
    #[structopt(long)]
    config: Option<String>,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    let opt = Opt::from_args();
    debug!("opt: {:?}", opt);

    let config_path = openrr_apps::utils::get_apps_robot_config(opt.config_path).unwrap();
    let config = if let Some(overwrite) = &opt.config {
        let s = &fs::read_to_string(&config_path)?;
        // check if the input is valid config.
        let _base: RobotConfig = toml::from_str(s)?;
        let mut edit: toml::Value = toml::from_str(s)?;
        openrr_config::overwrite(&mut edit, overwrite)?;
        RobotConfig::from_str(&toml::to_string(&edit)?, config_path)?
    } else {
        RobotConfig::try_new(config_path)?
    };

    openrr_apps::utils::init(env!("CARGO_BIN_NAME"), &config);
    let client: BoxRobotClient = config.create_robot_client()?;
    let robot = urdf_rs::read_file(config.openrr_clients_config.urdf_full_path().unwrap())?;
    joint_position_sender(client, robot)?;
    Ok(())
}
