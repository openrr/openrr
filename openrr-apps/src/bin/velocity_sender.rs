#![feature(no_coverage)]

use std::path::PathBuf;

use anyhow::Result;
use clap::Parser;
use openrr_apps::utils::init_tracing;
use openrr_client::BoxRobotClient;
use tracing::debug;

/// An openrr GUI tool to send base velocity.
#[derive(Parser, Debug)]
#[clap(name = env!("CARGO_BIN_NAME"))]
struct Opt {
    /// Path to the setting file.
    #[clap(short, long, value_parser)]
    config_path: Option<PathBuf>,
    /// Set options from command line. These settings take priority over the
    /// setting file specified by --config-path.
    #[clap(long)]
    config: Option<String>,
}

#[no_coverage]
fn main() -> Result<()> {
    init_tracing();
    let opt = Opt::parse();
    debug!("opt: {opt:?}");

    let config_path = openrr_apps::utils::get_apps_robot_config(opt.config_path);
    let config =
        openrr_apps::utils::resolve_robot_config(config_path.as_deref(), opt.config.as_deref())?;

    openrr_apps::utils::init(env!("CARGO_BIN_NAME"), &config);
    let client: BoxRobotClient = config.create_robot_client()?;
    openrr_gui::velocity_sender(client)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use clap::CommandFactory;

    use super::*;

    #[test]
    fn assert_app() {
        Opt::command().debug_assert();
    }
}
