use std::path::PathBuf;

use anyhow::Result;
use openrr_apps::utils::init_tracing;
use openrr_client::BoxRobotClient;
use openrr_gui::velocity_sender;
use structopt::StructOpt;
use tracing::debug;

/// An openrr GUI tool to send base velocity.
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
    init_tracing();
    let opt = Opt::from_args();
    debug!("opt: {:?}", opt);

    let config_path = openrr_apps::utils::get_apps_robot_config(opt.config_path);
    let config =
        openrr_apps::utils::resolve_robot_config(config_path.as_deref(), opt.config.as_deref())?;

    openrr_apps::utils::init(env!("CARGO_BIN_NAME"), &config);
    let client: BoxRobotClient = config.create_robot_client()?;
    velocity_sender(client)?;
    Ok(())
}
