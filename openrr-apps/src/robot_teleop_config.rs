use std::path::PathBuf;

use arci_gamepad_gilrs::GilGamepadConfig;
use openrr_client::resolve_relative_path;
use openrr_teleop::ControlNodesConfig;
use serde::{Deserialize, Serialize};

use crate::Error;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct RobotTeleopConfig {
    pub robot_config_path: String,
    robot_config_full_path: Option<PathBuf>,
    #[serde(default)]
    pub initial_mode: String,
    pub control_nodes_config: ControlNodesConfig,
    #[serde(default)]
    pub gil_gamepad_config: GilGamepadConfig,
}

impl RobotTeleopConfig {
    pub fn try_new<P: AsRef<std::path::Path>>(path: P) -> Result<Self, Error> {
        let mut config: RobotTeleopConfig = toml::from_str(
            &std::fs::read_to_string(&path)
                .map_err(|e| Error::NoFile(path.as_ref().to_owned(), e))?,
        )
        .map_err(|e| Error::TomlParseFailure(path.as_ref().to_owned(), e))?;
        config.robot_config_full_path = resolve_relative_path(path, &config.robot_config_path)?;
        Ok(config)
    }

    pub fn robot_config_full_path(&self) -> &Option<PathBuf> {
        &self.robot_config_full_path
    }
}
