use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};

use arci_gamepad_gilrs::GilGamepadConfig;
#[cfg(feature = "ros")]
use arci_ros::JoyGamepadConfig;
use openrr_client::resolve_relative_path;
use openrr_teleop::ControlNodesConfig;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::{resolve_plugin_path, Error};

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(rename_all = "kebab-case")]
pub enum BuiltinGamepad {
    Gilrs,
    Keyboard,
    JoyGamepad,
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(untagged)]
pub enum GamepadKind {
    Builtin(BuiltinGamepad),
    Plugin(String),
}

impl Default for GamepadKind {
    fn default() -> Self {
        Self::Builtin(BuiltinGamepad::Gilrs)
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TeleopPluginConfig {
    /// Path to the plugin. If no extension is specified, the default extension
    /// for `cdylib` on the current OS will be selected.
    /// (linux: `.so`, macos: `.dylib`, windows: `.dll`)
    pub path: PathBuf,
    /// Arguments passed when creating this instance.
    pub args: Option<String>,
    /// Pass the contents of the specified file as an argument.
    pub args_from_path: Option<PathBuf>,
}

#[derive(Debug, Serialize, Deserialize, Clone, Default, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RobotTeleopConfig {
    pub robot_config_path: Option<String>,
    robot_config_full_path: Option<PathBuf>,
    #[serde(default)]
    pub initial_mode: String,
    #[serde(default)]
    pub gamepad: GamepadKind,
    pub control_nodes_config: ControlNodesConfig,
    #[serde(default)]
    pub gil_gamepad_config: GilGamepadConfig,
    #[cfg(feature = "ros")]
    #[serde(default)]
    pub joy_gamepad_config: JoyGamepadConfig,
    #[serde(default)]
    pub joy_gamepad_config: JoyGamepadConfig,
    #[serde(default)]
    pub plugins: HashMap<String, TeleopPluginConfig>,
}

impl RobotTeleopConfig {
    pub fn new<P: AsRef<Path>>(path: P) -> Result<Self, Error> {
        Self::from_str(
            &std::fs::read_to_string(&path)
                .map_err(|e| Error::NoFile(path.as_ref().to_owned(), e))?,
            path,
        )
    }

    pub fn from_str<P: AsRef<Path>>(s: &str, path: P) -> Result<Self, Error> {
        let path = path.as_ref();
        let mut config: RobotTeleopConfig =
            toml::from_str(s).map_err(|e| Error::TomlParseFailure(path.to_owned(), e))?;
        config.robot_config_full_path = config
            .robot_config_path
            .as_ref()
            .map(|robot_config_path| resolve_relative_path(path, robot_config_path))
            .transpose()?;
        for plugin_config in config.plugins.values_mut() {
            resolve_plugin_path(&mut plugin_config.path, path)?;
            if let Some(args_path) = plugin_config.args_from_path.take() {
                plugin_config.args_from_path =
                    Some(openrr_client::resolve_relative_path(path, &args_path)?);
            }
        }
        Ok(config)
    }

    pub fn robot_config_full_path(&self) -> &Option<PathBuf> {
        &self.robot_config_full_path
    }
}
