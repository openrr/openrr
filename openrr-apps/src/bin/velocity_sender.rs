use std::{fs, path::PathBuf};

use anyhow::Result;
use openrr_apps::{utils::init_tracing, RobotConfig};
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
    let config = match (&config_path, &opt.config) {
        (Some(config_path), Some(overwrite)) => {
            let s = &fs::read_to_string(&config_path)?;
            let s = &openrr_config::overwrite_str(s, overwrite)?;
            RobotConfig::from_str(s, config_path)?
        }
        (Some(config_path), None) => RobotConfig::new(config_path)?,
        (None, overwrite) => {
            let mut config = RobotConfig::default();
            config
                .urdf_viz_clients_configs
                .push(arci_urdf_viz::UrdfVizWebClientConfig {
                    name: "all".into(),
                    joint_names: None,
                    wrap_with_joint_position_limiter: false,
                    wrap_with_joint_velocity_limiter: false,
                    joint_velocity_limits: None,
                    joint_position_limits: None,
                });
            if let Some(overwrite) = overwrite {
                let s = &toml::to_string(&config)?;
                let s = &openrr_config::overwrite_str(s, overwrite)?;
                config = toml::from_str(s)?;
            }
            config
        }
    };

    openrr_apps::utils::init(env!("CARGO_BIN_NAME"), &config);
    let client: BoxRobotClient = config.create_robot_client()?;
    velocity_sender(client)?;
    Ok(())
}
