use std::path::PathBuf;

use arci_gamepad_gilrs::GilGamepadConfig;
use openrr_client::resolve_relative_path;
use openrr_teleop::ControlNodesConfig;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::Error;

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
pub enum GamepadKind {
    Gilrs,
    Keyboard,
}

impl Default for GamepadKind {
    fn default() -> Self {
        Self::Gilrs
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, Default, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RobotTeleopConfig {
    pub robot_config_path: String,
    robot_config_full_path: Option<PathBuf>,
    #[serde(default)]
    pub initial_mode: String,
    #[serde(default)]
    pub gamepad: GamepadKind,
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
        config.robot_config_full_path =
            Some(resolve_relative_path(path, &config.robot_config_path)?);
        Ok(config)
    }

    pub fn robot_config_full_path(&self) -> &Option<PathBuf> {
        &self.robot_config_full_path
    }
}
