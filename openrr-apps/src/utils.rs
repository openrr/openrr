use std::{
    fs, io,
    path::{Path, PathBuf},
};

use rand::prelude::*;
use serde::Deserialize;
use tracing::{debug, warn, Event, Level, Subscriber};
use tracing_appender::{
    non_blocking::WorkerGuard,
    rolling::{RollingFileAppender, Rotation},
};
use tracing_subscriber::{
    fmt::{format::Format, FmtContext, FormatEvent, FormatFields, Layer},
    layer::SubscriberExt,
    registry::LookupSpan,
    EnvFilter,
};

use crate::{RobotConfig, RobotTeleopConfig};

const OPENRR_APPS_CONFIG_ENV_NAME: &str = "OPENRR_APPS_ROBOT_CONFIG_PATH";
const DEFAULT_JOINT_CLIENT_NAME: &str = "all";

/// Get robot config from input or env OPENRR_APPS_ROBOT_CONFIG_PATH
pub fn get_apps_robot_config(config: Option<PathBuf>) -> Option<PathBuf> {
    if config.is_some() {
        config
    } else {
        std::env::var(OPENRR_APPS_CONFIG_ENV_NAME)
            .map(|s| {
                warn!("### ENV VAR {} is used ###", s);
                PathBuf::from(s)
            })
            .ok()
    }
}

pub fn resolve_robot_config(
    config_path: Option<&Path>,
    overwrite: Option<&str>,
) -> anyhow::Result<RobotConfig> {
    match (config_path, overwrite) {
        (Some(config_path), Some(overwrite)) => {
            let s = &fs::read_to_string(&config_path)?;
            let s = &openrr_config::overwrite_str(s, overwrite)?;
            Ok(RobotConfig::from_str(s, config_path)?)
        }
        (Some(config_path), None) => Ok(RobotConfig::new(config_path)?),
        (None, overwrite) => {
            let mut config = RobotConfig::default();
            config
                .urdf_viz_clients_configs
                .push(arci_urdf_viz::UrdfVizWebClientConfig {
                    name: DEFAULT_JOINT_CLIENT_NAME.into(),
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
            Ok(config)
        }
    }
}

pub fn resolve_teleop_config(
    config_path: Option<&Path>,
    overwrite: Option<&str>,
) -> anyhow::Result<RobotTeleopConfig> {
    match (config_path, overwrite) {
        (Some(teleop_config_path), Some(overwrite)) => {
            let s = &fs::read_to_string(&teleop_config_path)?;
            let s = &openrr_config::overwrite_str(s, overwrite)?;
            Ok(RobotTeleopConfig::from_str(s, teleop_config_path)?)
        }
        (Some(teleop_config_path), None) => Ok(RobotTeleopConfig::new(teleop_config_path)?),
        (None, overwrite) => {
            let mut config = RobotTeleopConfig::default();
            config.control_nodes_config.move_base_mode = Some("base".into());
            if let Some(overwrite) = overwrite {
                let s = &toml::to_string(&config)?;
                let s = &openrr_config::overwrite_str(s, overwrite)?;
                config = toml::from_str(s)?;
            }
            Ok(config)
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_get_apps_robot_config() {
        let path = get_apps_robot_config(Some(PathBuf::from("a.toml")));
        assert!(path.is_some());
        assert_eq!(path.unwrap(), PathBuf::from("a.toml"));
        //
        std::env::set_var(OPENRR_APPS_CONFIG_ENV_NAME, "b.yaml");
        let path = get_apps_robot_config(Some(PathBuf::from("a.toml")));
        assert!(path.is_some());
        assert_eq!(path.unwrap(), PathBuf::from("a.toml"));
        std::env::remove_var(OPENRR_APPS_CONFIG_ENV_NAME);

        let path = get_apps_robot_config(None);
        assert!(path.is_none());

        std::env::set_var(OPENRR_APPS_CONFIG_ENV_NAME, "b.yaml");
        let path = get_apps_robot_config(None);
        assert!(path.is_some());
        assert_eq!(path.unwrap(), PathBuf::from("b.yaml"));
        std::env::remove_var(OPENRR_APPS_CONFIG_ENV_NAME);
    }
    #[test]
    fn test_log_config_default() {
        let default = LogConfig::default();
        assert_eq!(default.level, LogLevel::default());
        assert_eq!(default.rotation, LogRotation::default());
        assert_eq!(default.prefix, default_log_prefix());
    }
    #[test]
    fn test_openrr_formatter() {
        let config = LogConfig {
            directory: tempfile::tempdir().unwrap().path().to_path_buf(),
            ..Default::default()
        };
        {
            let _guard = init_tracing_with_file_appender(config.clone(), "prefix".to_string());
            tracing::info!("log");
        }
        assert!(config.directory.is_dir());
        for entry in std::fs::read_dir(config.directory).unwrap() {
            let log = std::fs::read_to_string(entry.unwrap().path()).unwrap();
            assert!(log.starts_with("prefix "));
        }
    }
}

/// Do something needed to start the program
pub fn init(name: &str, config: &RobotConfig) {
    #[cfg(feature = "ros")]
    if config.has_ros_clients() {
        arci_ros::init(name);
    }
    debug!("init {} with {:?}", name, config);
}

/// Do something needed to start the program for multiple
pub fn init_with_anonymize(name: &str, config: &RobotConfig) {
    let suffix: u64 = rand::thread_rng().gen();
    let anon_name = format!("{}_{}", name, suffix);
    init(&anon_name, config);
}

pub fn init_tracing() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .with_writer(io::stderr)
        .init();
}

#[derive(Clone)]
pub struct OpenrrFormatter {
    formatter: Format,
    name: String,
}

impl OpenrrFormatter {
    fn new(name: String) -> Self {
        Self {
            formatter: tracing_subscriber::fmt::format(),
            name,
        }
    }
}

impl<S, N> FormatEvent<S, N> for OpenrrFormatter
where
    S: Subscriber + for<'a> LookupSpan<'a>,
    N: for<'a> FormatFields<'a> + 'static,
{
    fn format_event(
        &self,
        ctx: &FmtContext<'_, S, N>,
        writer: &mut dyn std::fmt::Write,
        event: &Event<'_>,
    ) -> std::fmt::Result {
        write!(writer, "{} ", self.name)?;
        self.formatter.format_event(ctx, writer, event)
    }
}

pub fn init_tracing_with_file_appender(config: LogConfig, name: String) -> WorkerGuard {
    let default_level = match config.level {
        LogLevel::TRACE => Level::TRACE,
        LogLevel::DEBUG => Level::DEBUG,
        LogLevel::INFO => Level::INFO,
        LogLevel::WARN => Level::WARN,
        LogLevel::ERROR => Level::ERROR,
    };
    let rotation = match config.rotation {
        LogRotation::MINUTELY => Rotation::MINUTELY,
        LogRotation::HOURLY => Rotation::HOURLY,
        LogRotation::DAILY => Rotation::DAILY,
        LogRotation::NEVER => Rotation::NEVER,
    };
    let formatter = OpenrrFormatter::new(name);
    let file_appender = RollingFileAppender::new(rotation, config.directory, config.prefix);
    let (file_writer, guard) = tracing_appender::non_blocking(file_appender);
    let subscriber = tracing_subscriber::registry()
        .with(EnvFilter::from_default_env().add_directive(default_level.into()))
        .with(
            Layer::new()
                .event_format(formatter.clone())
                .with_writer(io::stderr),
        )
        .with(
            Layer::new()
                .event_format(formatter)
                .with_writer(file_writer),
        );
    tracing::subscriber::set_global_default(subscriber).expect("Unable to set a global collector");
    guard
}

#[derive(Deserialize, Debug, PartialEq, Clone)]
pub enum LogLevel {
    TRACE,
    DEBUG,
    INFO,
    WARN,
    ERROR,
}

impl Default for LogLevel {
    fn default() -> Self {
        Self::INFO
    }
}

#[derive(Deserialize, PartialEq, Debug, Clone)]
pub enum LogRotation {
    MINUTELY,
    HOURLY,
    DAILY,
    NEVER,
}

impl Default for LogRotation {
    fn default() -> Self {
        Self::HOURLY
    }
}

#[derive(Deserialize, PartialEq, Clone)]
pub struct LogConfig {
    #[serde(default)]
    pub level: LogLevel,
    #[serde(default)]
    pub rotation: LogRotation,
    #[serde(default = "default_log_directory")]
    pub directory: PathBuf,
    #[serde(default = "default_log_prefix")]
    pub prefix: PathBuf,
}

impl Default for LogConfig {
    fn default() -> Self {
        Self {
            directory: default_log_directory(),
            prefix: default_log_prefix(),
            level: LogLevel::default(),
            rotation: LogRotation::default(),
        }
    }
}

fn default_log_prefix() -> PathBuf {
    PathBuf::from("log")
}

fn default_log_directory() -> PathBuf {
    std::env::temp_dir()
}
