use std::path::PathBuf;

use openrr_apps::{Error, RobotConfig};
use openrr_command::{RobotCommand, RobotCommandExecutor};
use structopt::StructOpt;
use tracing::info;

#[derive(StructOpt, Debug)]
#[structopt(
    name = env!("CARGO_BIN_NAME"),
    about = "An openrr command line tool."
)]
struct RobotCommandArgs {
    /// Path to the setting file.
    #[structopt(short, long, parse(from_os_str))]
    config_path: Option<PathBuf>,
    #[structopt(subcommand)]
    command: Option<RobotCommand>,
    /// Prints the default setting as TOML.
    #[structopt(long)]
    show_default_config: bool,
}

#[tokio::main]
async fn main() -> Result<(), Error> {
    tracing_subscriber::fmt::init();
    let args = RobotCommandArgs::from_args();
    info!("ParsedArgs {:?}", args);

    if args.show_default_config {
        print!("{}", toml::to_string(&RobotConfig::default()).unwrap());
        return Ok(());
    }

    if let Some(config_path) = openrr_apps::utils::get_apps_robot_config(args.config_path) {
        let command = args.command.ok_or(Error::NoCommand)?;
        let robot_config = RobotConfig::try_new(config_path)?;
        openrr_apps::utils::init_with_anonymize(env!("CARGO_BIN_NAME"), &robot_config);
        let client = robot_config.create_robot_client()?;
        let executor = RobotCommandExecutor {};
        Ok(executor.execute(&client, &command).await?)
    } else {
        Err(Error::NoConfigPath)
    }
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
