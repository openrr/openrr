use std::{
    fs, io,
    path::{Path, PathBuf},
};

use rand::prelude::*;
use tracing::{debug, warn};

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
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_writer(io::stderr)
        .init();
}
