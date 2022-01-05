use std::path::PathBuf;

use anyhow::Result;
use clap::Parser;
use openrr_apps::utils::init_tracing;
use openrr_client::BoxRobotClient;
use openrr_gui::joint_position_sender;
use tracing::debug;

/// An openrr GUI tool to send joint positions.
#[derive(Parser, Debug)]
#[clap(name = env!("CARGO_BIN_NAME"))]
struct Opt {
    /// Path to the setting file.
    #[clap(short, long, parse(from_os_str))]
    config_path: Option<PathBuf>,
    /// Set options from command line. These settings take priority over the
    /// setting file specified by --config-path.
    #[clap(long)]
    config: Option<String>,
}

fn main() -> Result<()> {
    init_tracing();
    let opt = Opt::parse();
    debug!("opt: {:?}", opt);

    let config_path = openrr_apps::utils::get_apps_robot_config(opt.config_path);
    let config =
        openrr_apps::utils::resolve_robot_config(config_path.as_deref(), opt.config.as_deref())?;

    openrr_apps::utils::init(env!("CARGO_BIN_NAME"), &config);
    let client: BoxRobotClient = config.create_robot_client()?;
    let robot = match config.openrr_clients_config.urdf_full_path() {
        Some(path) => urdf_rs::utils::read_urdf_or_xacro(path)?,
        None => arci_urdf_viz::UrdfVizWebClient::default().get_urdf()?,
    };
    joint_position_sender(client, robot)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use clap::IntoApp;

    use super::*;

    #[test]
    fn assert_app() {
        Opt::into_app().debug_assert();
    }
}
