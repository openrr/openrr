use std::{fs, path::PathBuf};

use anyhow::Result;
use clap::{ArgEnum, Parser};
use openrr_apps::utils::init_tracing;
use schemars::schema_for;
use serde::Deserialize;
use tracing::debug;

#[derive(Debug, Parser)]
#[clap(name = env!("CARGO_BIN_NAME"))]
struct Args {
    #[clap(subcommand)]
    subcommand: Subcommand,
}

#[derive(Debug, Parser)]
enum Subcommand {
    /// Generate JSON schema for the specified config file.
    Schema {
        /// Kind of config file.
        #[clap(arg_enum, ignore_case = true)]
        kind: ConfigKind,
    },
    Merge {
        /// Path to the setting file.
        #[clap(long, parse(from_os_str))]
        config_path: PathBuf,
        /// Config to overwrite
        #[clap(long)]
        config: String,
    },
}

#[derive(Debug, Clone, Copy, ArgEnum)]
enum ConfigKind {
    RobotConfig,
    RobotTeleopConfig,
}

#[derive(Debug, Deserialize)]
#[serde(untagged)]
enum Config {
    RobotConfig(Box<openrr_apps::RobotConfig>),
    RobotTeleopConfig(Box<openrr_apps::RobotTeleopConfig>),
}

fn main() -> Result<()> {
    init_tracing();
    let args = Args::parse();
    debug!(?args);

    match args.subcommand {
        Subcommand::Schema { kind } => {
            let schema = match kind {
                ConfigKind::RobotConfig => schema_for!(openrr_apps::RobotConfig),
                ConfigKind::RobotTeleopConfig => schema_for!(openrr_apps::RobotTeleopConfig),
            };
            println!("{}", serde_json::to_string_pretty(&schema).unwrap());
        }
        Subcommand::Merge {
            config_path,
            config: overwrite,
        } => {
            let s = &fs::read_to_string(&config_path)?;
            let s = &openrr_config::overwrite_str(s, &overwrite)?;
            // check if the edited document is valid config.
            let _base: Config = toml::from_str(s)?;
            println!("{s}");
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use clap::CommandFactory;

    use super::*;

    #[test]
    fn assert_app() {
        Args::command().debug_assert();
    }
}
