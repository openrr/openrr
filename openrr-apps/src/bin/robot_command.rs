use std::{fs, path::PathBuf};

use anyhow::Result;
use openrr_apps::{Error, RobotConfig};
use openrr_command::{RobotCommand, RobotCommandExecutor};
use structopt::StructOpt;
use tracing::info;

/// An openrr command line tool.
#[derive(StructOpt, Debug)]
#[structopt(name = env!("CARGO_BIN_NAME"))]
struct RobotCommandArgs {
    /// Path to the setting file.
    #[structopt(short, long, parse(from_os_str))]
    config_path: Option<PathBuf>,
    /// Set options from command line. These settings take priority over the
    /// setting file specified by --config-path.
    #[structopt(long)]
    config: Option<String>,
    #[structopt(subcommand)]
    command: Option<RobotCommand>,
    /// Prints the default setting as TOML.
    #[structopt(long)]
    show_default_config: bool,
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    let args = RobotCommandArgs::from_args();
    info!("ParsedArgs {:?}", args);

    if args.show_default_config {
        print!("{}", toml::to_string(&RobotConfig::default()).unwrap());
        return Ok(());
    }

    let config_path =
        openrr_apps::utils::get_apps_robot_config(args.config_path).ok_or(Error::NoConfigPath)?;
    let command = args.command.ok_or(Error::NoCommand)?;
    let robot_config = if let Some(overwrite) = &args.config {
        let s = &fs::read_to_string(&config_path)?;
        let s = &openrr_config::overwrite_str(s, overwrite)?;
        RobotConfig::from_str(s, config_path)?
    } else {
        RobotConfig::try_new(config_path)?
    };

    openrr_apps::utils::init_with_anonymize(env!("CARGO_BIN_NAME"), &robot_config);
    let client = robot_config.create_robot_client()?;
    let executor = RobotCommandExecutor {};
    Ok(executor.execute(&client, &command).await?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_args() {
        let bin = env!("CARGO_BIN_NAME");
        assert!(RobotCommandArgs::from_iter_safe(&[bin]).is_ok());
        assert!(RobotCommandArgs::from_iter_safe(&[bin, "--show-default-config"]).is_ok());
        assert!(RobotCommandArgs::from_iter_safe(&[bin, "--config-path", "path", "list"]).is_ok());
        assert!(RobotCommandArgs::from_iter_safe(&[
            bin,
            "--show-default-config",
            "--config-path",
            "path"
        ])
        .is_ok());
    }
}
