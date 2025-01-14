use std::{
    collections::HashMap,
    env, fmt, fs,
    ops::Not,
    path::{Path, PathBuf},
    sync::Arc,
};

use anyhow::format_err;
use arci::{JointTrajectoryClient, Localization, MoveBase, Navigation, Speaker};
#[cfg(feature = "ros")]
use arci_ros::{
    RosCmdVelMoveBase, RosCmdVelMoveBaseConfig, RosControlActionClientConfig,
    RosControlClientConfig, RosEspeakClient, RosEspeakClientConfig, RosLocalizationClient,
    RosLocalizationClientConfig, RosNavClient, RosNavClientConfig,
};
use arci_speak_audio::AudioSpeaker;
use arci_speak_cmd::LocalCommand;
use arci_urdf_viz::{UrdfVizWebClient, UrdfVizWebClientConfig};
use openrr_client::{OpenrrClientsConfig, PrintSpeaker, RobotClient};
use openrr_plugin::PluginProxy;
use openrr_tracing::Tracing;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::{debug, error, info};

use crate::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "kebab-case")]
pub enum BuiltinClient {
    /// [ROS1](https://ros.org)
    Ros,
    /// [urdf-viz](https://github.com/openrr/urdf-viz)
    UrdfViz,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum ClientKind {
    // Use builtin client, ros or urdf-viz.
    Builtin(BuiltinClient),
    // Use plugin.
    Plugin(String),
    // true: auto-selection
    // false: disable
    Auto(bool),
}

impl ClientKind {
    /// Returns true if the use of the ros client is explicitly specified.
    ///
    /// This is always false when ros feature is disabled. (ensured by validate_ros_config)
    #[cfg(feature = "ros")]
    fn is_builtin_ros(&self) -> bool {
        matches!(self, Self::Builtin(BuiltinClient::Ros))
    }

    fn is_auto(&self) -> bool {
        matches!(self, Self::Auto(true))
    }
}

impl Default for ClientKind {
    fn default() -> Self {
        Self::Auto(true)
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "type", content = "args")]
#[serde(deny_unknown_fields)]
#[non_exhaustive] // The variants will increase depending on the feature flag.
pub enum SpeakConfig {
    #[default]
    Print,
    Command,
    #[cfg(feature = "ros")]
    RosEspeak {
        config: RosEspeakClientConfig,
    },
    // Not public API.
    // A dummy variant to catch that there is a config that requires the ros feature.
    #[doc(hidden)]
    #[cfg(not(feature = "ros"))]
    RosEspeak {
        #[schemars(schema_with = "unimplemented_schema")]
        config: toml::Value,
    },
    Audio {
        map: HashMap<String, PathBuf>,
    },
}

impl SpeakConfig {
    pub fn build(&self) -> Result<Box<dyn Speaker>, Error> {
        Ok(match self {
            SpeakConfig::RosEspeak { config } => self.create_ros_espeak_client(config),
            SpeakConfig::Audio { map } => self.create_audio_speaker(map.clone())?,
            SpeakConfig::Command => self.create_local_command_speaker(),
            SpeakConfig::Print => self.create_print_speaker(),
        })
    }

    fn create_print_speaker(&self) -> Box<dyn Speaker> {
        Box::new(arci::Lazy::new(move || {
            debug!("create_print_speaker: creating PrintSpeaker");
            Ok(PrintSpeaker::new())
        }))
    }

    fn create_local_command_speaker(&self) -> Box<dyn Speaker> {
        Box::new(arci::Lazy::new(move || {
            debug!("create_local_command_speaker: creating LocalCommand");
            Ok(LocalCommand::new())
        }))
    }

    fn create_audio_speaker(
        &self,
        mut hash_map: HashMap<String, PathBuf>,
    ) -> Result<Box<dyn Speaker>, Error> {
        for path in hash_map.values_mut() {
            *path = openrr_config::evaluate(path.to_str().unwrap(), None)
                .map_err(arci::Error::Other)?
                .into();
        }
        Ok(Box::new(arci::Lazy::new(move || {
            debug!("create_audio_speaker: creating AudioSpeaker");
            Ok(AudioSpeaker::new(hash_map))
        })))
    }

    #[cfg(feature = "ros")]
    fn create_ros_espeak_client(&self, config: &RosEspeakClientConfig) -> Box<dyn Speaker> {
        let topic = config.topic.to_string();
        Box::new(arci::Lazy::new(move || {
            debug!("create_ros_espeak_client: creating RosEspeakClient");
            Ok(RosEspeakClient::new(&topic))
        }))
    }

    #[cfg(not(feature = "ros"))]
    fn create_ros_espeak_client(&self, _config: &toml::Value) -> Box<dyn Speaker> {
        unreachable!()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PluginConfig {
    /// Path to the plugin. If no extension is specified, the default extension
    /// for `cdylib` on the current OS will be selected.
    /// (linux: `.so`, macos: `.dylib`, windows: `.dll`)
    pub path: PathBuf,
    pub instances: Vec<PluginInstance>,
}

impl PluginConfig {
    fn find_instances_by_name<'a>(
        map: &'a HashMap<String, Self>,
        instance_name: &'a str,
        instance_kind: PluginInstanceKind,
    ) -> impl Iterator<Item = (&'a str, &'a PluginInstance)> {
        map.iter().flat_map(move |(plugin_name, plugin_config)| {
            plugin_config
                .instances
                .iter()
                .filter(move |instance| {
                    instance.name == instance_name && instance.type_ == instance_kind
                })
                .map(move |instance| (plugin_name.as_str(), instance))
        })
    }

    fn find_instances_by_kind(
        map: &HashMap<String, Self>,
        instance_kind: PluginInstanceKind,
    ) -> impl Iterator<Item = (&str, &PluginInstance)> {
        map.iter().flat_map(move |(plugin_name, plugin_config)| {
            plugin_config
                .instances
                .iter()
                .filter(move |instance| instance.type_ == instance_kind)
                .map(move |instance| (plugin_name.as_str(), instance))
        })
    }

    fn resolve_instance<'a>(
        map: &'a HashMap<String, Self>,
        instance_name: Option<&'a str>,
        instance_kind: PluginInstanceKind,
    ) -> Result<(&'a str, &'a PluginInstance), Error> {
        let instances: Vec<_> = if let Some(instance_name) = instance_name {
            Self::find_instances_by_name(map, instance_name, instance_kind).collect()
        } else {
            Self::find_instances_by_kind(map, instance_kind).collect()
        };

        if instances.is_empty() {
            return Err(Error::NoPluginInstance {
                name: instance_name.unwrap_or_default().to_string(),
                kind: instance_kind.to_string(),
            });
        }
        if instances.len() == 1 {
            return Ok(instances[0]);
        }

        if let Some(instance_name) = instance_name {
            Err(Error::DuplicateInstance(format!(
                "Multiple {instance_kind:?} plugin instances {instance_name:?} are found. Consider renaming one of the instances",
            )))
        } else {
            Err(Error::DuplicateInstance(format!(
                "Multiple plugin instances for {instance_kind:?} are found. Consider specifying the instance to use",
            )))
        }
    }
}

#[allow(clippy::needless_borrows_for_generic_args)] // https://github.com/rust-lang/rust-clippy/issues/13170
pub(crate) fn resolve_plugin_path(
    plugin_path: &mut PathBuf,
    base_path: impl AsRef<Path>,
) -> Result<(), Error> {
    *plugin_path = openrr_client::resolve_relative_path(base_path, &plugin_path)?;
    if plugin_path.extension().is_none() {
        plugin_path.set_extension(env::consts::DLL_EXTENSION);
    }
    Ok(())
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PluginInstance {
    /// Name of this plugin instance.
    pub name: String,
    /// Trait kind of this instance.
    #[serde(rename = "type")]
    pub type_: PluginInstanceKind,
    /// Arguments passed when creating this instance.
    pub args: Option<String>,
    /// Pass the contents of the specified file as an argument.
    pub args_from_path: Option<PathBuf>,
}

impl PluginInstance {
    pub fn load_args(&self) -> Result<String, Error> {
        if let Some(path) = &self.args_from_path {
            fs::read_to_string(path).map_err(|e| Error::NoFile(path.to_owned(), e))
        } else {
            Ok(self.args.clone().unwrap_or_default())
        }
    }

    fn create_lazy_instance<T, F>(
        &self,
        plugins: &mut PluginMap,
        plugin_name: &str,
        f: F,
    ) -> Result<arci::Lazy<'static, T>, Error>
    where
        T: fmt::Debug,
        F: FnOnce(&PluginProxy, String) -> Result<Option<T>, arci::Error> + Send + Sync + 'static,
    {
        let plugin = if let Some(plugin) = plugins.load(plugin_name)? {
            plugin
        } else {
            return Err(Error::NoPluginInstance {
                name: plugin_name.to_string(),
                kind: self.type_.to_string(),
            });
        };
        let args = self.load_args()?;
        let plugin_name = plugin_name.to_string();
        let instance_name = self.name.clone();
        let instance_kind = self.type_;
        Ok(arci::Lazy::new(move || match f(&plugin, args) {
            Ok(Some(instance)) => {
                info!(
                    "created `{instance_kind:?}` instance `{instance_name}` from plugin `{plugin_name}`",
                );
                Ok(instance)
            }
            res => instance_create_error(res, instance_kind, instance_name, plugin_name)?,
        }))
    }
}

/// Trait kind of the instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
#[non_exhaustive]
pub enum PluginInstanceKind {
    JointTrajectoryClient,
    Localization,
    MoveBase,
    Navigation,
    Speaker,
}

impl fmt::Display for PluginInstanceKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

#[derive(Debug, Default)]
pub struct PluginMap {
    path: HashMap<String, PathBuf>,
    cache: HashMap<String, Arc<PluginProxy>>,
}

impl PluginMap {
    pub fn load(&mut self, name: impl AsRef<str>) -> Result<Option<Arc<PluginProxy>>, arci::Error> {
        let name = name.as_ref();
        if let Some((name, path)) = self.path.remove_entry(name) {
            let plugin = Arc::new(PluginProxy::from_path(path)?);
            self.cache.insert(name, plugin.clone());
            Ok(Some(plugin))
        } else {
            Ok(self.cache.get(name).cloned())
        }
    }
}

impl<S: Into<String>, P: Into<PathBuf>> FromIterator<(S, P)> for PluginMap {
    fn from_iter<T: IntoIterator<Item = (S, P)>>(iter: T) -> Self {
        let path: HashMap<_, _> = iter
            .into_iter()
            .map(|(name, path)| (name.into(), path.into()))
            .collect();
        Self {
            cache: HashMap::with_capacity(path.len() / 2),
            path,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
#[non_exhaustive] // The fields will increase depending on the feature flag.
pub struct RobotConfig {
    // TOML format has a restriction that if a table itself contains tables,
    // all keys with non-table values must be emitted first.
    // Therefore, these fields must be located at the start of the struct.
    /// Joint trajectory clients to be used.
    pub joint_trajectory_clients: Option<Vec<String>>,
    /// Speakers to be used.
    pub speakers: Option<Vec<String>>,
    /// Localization to be used. `"ros"`, `"urdf-viz"`, `false`, or plugin instance name.
    #[serde(skip_serializing_if = "ClientKind::is_auto")]
    #[serde(default)]
    pub localization: ClientKind,
    /// MoveBase to be used. `"ros"`, `"urdf-viz"`, `false`, or plugin instance name.
    #[serde(skip_serializing_if = "ClientKind::is_auto")]
    #[serde(default)]
    pub move_base: ClientKind,
    /// Navigation to be used. `"ros"`, `"urdf-viz"`, `false`, or plugin instance name.
    #[serde(skip_serializing_if = "ClientKind::is_auto")]
    #[serde(default)]
    pub navigation: ClientKind,

    #[cfg(feature = "ros")]
    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub ros_clients_configs: Vec<RosControlClientConfig>,
    // A dummy field to catch that there is a config that requires the ros feature.
    #[cfg(not(feature = "ros"))]
    #[serde(default)]
    #[serde(skip_serializing_if = "Vec::is_empty")]
    #[schemars(schema_with = "unimplemented_schema")]
    ros_clients_configs: Vec<toml::Value>,

    #[cfg(feature = "ros")]
    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub ros_action_clients_configs: Vec<RosControlActionClientConfig>,
    // A dummy field to catch that there is a config that requires the ros feature.
    #[cfg(not(feature = "ros"))]
    #[serde(default)]
    #[serde(skip_serializing_if = "Vec::is_empty")]
    #[schemars(schema_with = "unimplemented_schema")]
    ros_action_clients_configs: Vec<toml::Value>,

    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub urdf_viz_clients_configs: Vec<UrdfVizWebClientConfig>,

    #[serde(default)]
    pub speak_configs: HashMap<String, SpeakConfig>,

    #[cfg(feature = "ros")]
    pub ros_cmd_vel_move_base_client_config: Option<RosCmdVelMoveBaseConfig>,
    // A dummy field to catch that there is a config that requires the ros feature.
    #[cfg(not(feature = "ros"))]
    #[schemars(schema_with = "unimplemented_schema")]
    ros_cmd_vel_move_base_client_config: Option<toml::Value>,

    #[cfg(feature = "ros")]
    pub ros_navigation_client_config: Option<RosNavClientConfig>,
    // A dummy field to catch that there is a config that requires the ros feature.
    #[cfg(not(feature = "ros"))]
    #[schemars(schema_with = "unimplemented_schema")]
    ros_navigation_client_config: Option<toml::Value>,

    #[cfg(feature = "ros")]
    pub ros_localization_client_config: Option<RosLocalizationClientConfig>,
    // A dummy field to catch that there is a config that requires the ros feature.
    #[cfg(not(feature = "ros"))]
    #[schemars(schema_with = "unimplemented_schema")]
    ros_localization_client_config: Option<toml::Value>,

    #[serde(default)]
    pub openrr_clients_config: OpenrrClientsConfig,

    #[serde(default)]
    pub openrr_tracing_config: OpenrrTracingConfig,

    #[serde(default)]
    pub plugins: HashMap<String, PluginConfig>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct OpenrrTracingConfig {
    /// Enable openrr-tracing for Localization. `true or `false` (default to `false`).
    #[serde(skip_serializing_if = "Not::not")]
    #[serde(default)]
    pub localization: bool,
    /// Enable openrr-tracing for MoveBase. `true or `false` (default to `false`).
    #[serde(skip_serializing_if = "Not::not")]
    #[serde(default)]
    pub move_base: bool,
    /// Enable openrr-tracing for Navigation. `true or `false` (default to `false`).
    #[serde(skip_serializing_if = "Not::not")]
    #[serde(default)]
    pub navigation: bool,
}

// Creates dummy schema for dummy fields.
#[cfg(not(feature = "ros"))]
fn unimplemented_schema(_gen: &mut schemars::gen::SchemaGenerator) -> schemars::schema::Schema {
    unimplemented!()
}

impl RobotConfig {
    const DEFAULT_SPEAKER_NAME: &'static str = "Default";

    /// Returns true if ros-related configs exist, or it is explicitly specified
    /// that the ros client is to be used.
    ///
    /// This is always false when ros feature is disabled.
    #[cfg(feature = "ros")]
    pub fn has_ros_clients(&self) -> bool {
        let mut has_ros_clients = false;
        let speak_configs = self.speak_configs.clone();
        for (_, speak_config) in speak_configs {
            has_ros_clients |= matches!(speak_config, SpeakConfig::RosEspeak { .. });
        }
        has_ros_clients |= !self.ros_clients_configs.is_empty();
        has_ros_clients |= !self.ros_action_clients_configs.is_empty();
        has_ros_clients |= self.move_base.is_builtin_ros()
            || (self.move_base.is_auto() && self.ros_cmd_vel_move_base_client_config.is_some());
        has_ros_clients |= self.navigation.is_builtin_ros()
            || (self.navigation.is_auto() && self.ros_navigation_client_config.is_some());
        has_ros_clients |= self.localization.is_builtin_ros()
            || (self.localization.is_auto() && self.ros_localization_client_config.is_some());
        has_ros_clients
    }

    #[cfg(not(feature = "ros"))]
    pub fn has_ros_clients(&self) -> bool {
        false
    }

    pub fn create_robot_client<L, M, N>(&self) -> Result<RobotClient<L, M, N>, Error>
    where
        L: Localization + From<Box<dyn Localization>>,
        M: MoveBase + From<Box<dyn MoveBase>>,
        N: Navigation + From<Box<dyn Navigation>>,
    {
        let mut plugins: PluginMap = self
            .plugins
            .iter()
            .map(|(plugin_name, config)| (plugin_name, &config.path))
            .collect();

        let joint_trajectory_clients = self.create_raw_joint_trajectory_clients(&mut plugins)?;
        let speakers = self.create_speakers(&mut plugins)?;
        let localization = self.create_localization(&mut plugins)?;
        let move_base = self.create_move_base(&mut plugins)?;
        let navigation = self.create_navigation(&mut plugins)?;

        Ok(RobotClient::new(
            self.openrr_clients_config.clone(),
            joint_trajectory_clients,
            speakers,
            localization.map(L::from),
            move_base.map(M::from),
            navigation.map(N::from),
        )?)
    }

    fn create_localization_urdf_viz(&self) -> Box<dyn Localization> {
        let loc = arci::Lazy::new(move || {
            debug!("create_localization_urdf_viz: creating UrdfVizWebClient");
            Ok(UrdfVizWebClient::default())
        });
        if self.openrr_tracing_config.localization {
            Box::new(Tracing::new(loc))
        } else {
            Box::new(loc)
        }
    }

    #[cfg(feature = "ros")]
    fn create_localization_ros(&self) -> Option<Box<dyn Localization>> {
        let config = self.ros_localization_client_config.clone()?;
        let loc = arci::Lazy::new(move || {
            debug!("create_localization_ros: creating RosLocalizationClient");
            Ok(RosLocalizationClient::new_from_config(config))
        });
        if self.openrr_tracing_config.localization {
            Some(Box::new(Tracing::new(loc)))
        } else {
            Some(Box::new(loc))
        }
    }

    #[cfg(not(feature = "ros"))]
    fn create_localization_ros(&self) -> Option<Box<dyn Localization>> {
        unreachable!()
    }

    fn create_localization(
        &self,
        plugins: &mut PluginMap,
    ) -> Result<Option<Box<dyn Localization>>, Error> {
        let (plugin_name, instance) = match &self.localization {
            ClientKind::Auto(false) => return Ok(None),
            ClientKind::Auto(true) => {
                if self.ros_localization_client_config.is_some() {
                    return Ok(self.create_localization_ros());
                }
                match PluginConfig::resolve_instance(
                    &self.plugins,
                    None,
                    PluginInstanceKind::Localization,
                ) {
                    Err(Error::NoPluginInstance { .. }) => {
                        // If ros is already used, it would *not* usually be
                        // assumed that urdf-viz would also be used.
                        // Users who want to use both at the same time need to
                        // specify it explicitly by `localization = "urdf-viz"`.
                        if self.has_ros_clients() {
                            return Ok(None);
                        }
                        return Ok(Some(self.create_localization_urdf_viz()));
                    }
                    res => res?,
                }
            }
            ClientKind::Builtin(BuiltinClient::Ros) => {
                return Ok(self.create_localization_ros());
            }
            ClientKind::Builtin(BuiltinClient::UrdfViz) => {
                return Ok(Some(self.create_localization_urdf_viz()));
            }
            ClientKind::Plugin(instance_name) => PluginConfig::resolve_instance(
                &self.plugins,
                Some(instance_name),
                PluginInstanceKind::Localization,
            )?,
        };

        Ok(Some(Box::new(instance.create_lazy_instance(
            plugins,
            plugin_name,
            PluginProxy::new_localization,
        )?)))
    }

    fn create_navigation_urdf_viz(&self) -> Box<dyn Navigation> {
        let nav = arci::Lazy::new(move || {
            debug!("create_navigation_urdf_viz: creating UrdfVizWebClient");
            Ok(UrdfVizWebClient::default())
        });
        if self.openrr_tracing_config.navigation {
            Box::new(Tracing::new(nav))
        } else {
            Box::new(nav)
        }
    }

    #[cfg(feature = "ros")]
    fn create_navigation_ros(&self) -> Option<Box<dyn Navigation>> {
        let config = self.ros_navigation_client_config.clone()?;
        let nav = arci::Lazy::new(move || {
            debug!("create_navigation_ros: creating RosNavClient");
            Ok(RosNavClient::new_from_config(config))
        });
        if self.openrr_tracing_config.navigation {
            Some(Box::new(Tracing::new(nav)))
        } else {
            Some(Box::new(nav))
        }
    }

    #[cfg(not(feature = "ros"))]
    fn create_navigation_ros(&self) -> Option<Box<dyn Navigation>> {
        unreachable!()
    }

    fn create_navigation(
        &self,
        plugins: &mut PluginMap,
    ) -> Result<Option<Box<dyn Navigation>>, Error> {
        let (plugin_name, instance) = match &self.navigation {
            ClientKind::Auto(false) => return Ok(None),
            ClientKind::Auto(true) => {
                if self.ros_navigation_client_config.is_some() {
                    return Ok(self.create_navigation_ros());
                }
                match PluginConfig::resolve_instance(
                    &self.plugins,
                    None,
                    PluginInstanceKind::Navigation,
                ) {
                    Err(Error::NoPluginInstance { .. }) => {
                        // If ros is already used, it would *not* usually be
                        // assumed that urdf-viz would also be used.
                        // Users who want to use both at the same time need to
                        // specify it explicitly by `navigation = "urdf-viz"`.
                        if self.has_ros_clients() {
                            return Ok(None);
                        }
                        return Ok(Some(self.create_navigation_urdf_viz()));
                    }
                    res => res?,
                }
            }
            ClientKind::Builtin(BuiltinClient::Ros) => {
                return Ok(self.create_navigation_ros());
            }
            ClientKind::Builtin(BuiltinClient::UrdfViz) => {
                return Ok(Some(self.create_navigation_urdf_viz()));
            }
            ClientKind::Plugin(instance_name) => PluginConfig::resolve_instance(
                &self.plugins,
                Some(instance_name),
                PluginInstanceKind::Navigation,
            )?,
        };

        Ok(Some(Box::new(instance.create_lazy_instance(
            plugins,
            plugin_name,
            PluginProxy::new_navigation,
        )?)))
    }

    fn create_move_base_urdf_viz(&self) -> Box<dyn MoveBase> {
        let base = arci::Lazy::new(move || {
            debug!("create_move_base_urdf_viz: creating UrdfVizWebClient");
            let urdf_viz_client = UrdfVizWebClient::default();
            urdf_viz_client.run_send_velocity_thread();
            Ok(urdf_viz_client)
        });
        if self.openrr_tracing_config.move_base {
            Box::new(Tracing::new(base))
        } else {
            Box::new(base)
        }
    }

    #[cfg(feature = "ros")]
    fn create_move_base_ros(&self) -> Option<Box<dyn MoveBase>> {
        let topic = self
            .ros_cmd_vel_move_base_client_config
            .as_ref()?
            .topic
            .to_string();
        let base = arci::Lazy::new(move || {
            debug!("create_move_base_ros: creating RosCmdVelMoveBase");
            Ok(RosCmdVelMoveBase::new(&topic))
        });
        if self.openrr_tracing_config.move_base {
            Some(Box::new(Tracing::new(base)))
        } else {
            Some(Box::new(base))
        }
    }

    #[cfg(not(feature = "ros"))]
    fn create_move_base_ros(&self) -> Option<Box<dyn MoveBase>> {
        unreachable!()
    }

    fn create_move_base(
        &self,
        plugins: &mut PluginMap,
    ) -> Result<Option<Box<dyn MoveBase>>, Error> {
        let (plugin_name, instance) = match &self.move_base {
            ClientKind::Auto(false) => return Ok(None),
            ClientKind::Auto(true) => {
                if self.ros_cmd_vel_move_base_client_config.is_some() {
                    return Ok(self.create_move_base_ros());
                }
                match PluginConfig::resolve_instance(
                    &self.plugins,
                    None,
                    PluginInstanceKind::MoveBase,
                ) {
                    Err(Error::NoPluginInstance { .. }) => {
                        // If ros is already used, it would *not* usually be
                        // assumed that urdf-viz would also be used.
                        // Users who want to use both at the same time need to
                        // specify it explicitly by `move_base = "urdf-viz"`.
                        if self.has_ros_clients() {
                            return Ok(None);
                        }
                        return Ok(Some(self.create_move_base_urdf_viz()));
                    }
                    res => res?,
                }
            }
            ClientKind::Builtin(BuiltinClient::Ros) => {
                return Ok(self.create_move_base_ros());
            }
            ClientKind::Builtin(BuiltinClient::UrdfViz) => {
                return Ok(Some(self.create_move_base_urdf_viz()));
            }
            ClientKind::Plugin(instance_name) => PluginConfig::resolve_instance(
                &self.plugins,
                Some(instance_name),
                PluginInstanceKind::MoveBase,
            )?,
        };

        Ok(Some(Box::new(instance.create_lazy_instance(
            plugins,
            plugin_name,
            PluginProxy::new_move_base,
        )?)))
    }

    fn create_speakers(
        &self,
        plugins: &mut PluginMap,
    ) -> Result<HashMap<String, Arc<dyn Speaker>>, Error> {
        let mut speakers: HashMap<_, Arc<dyn Speaker>> = HashMap::new();
        for (name, speak_config) in self
            .speak_configs
            .iter()
            .filter(|(name, _)| self.speakers.as_ref().is_none_or(|v| v.contains(name)))
        {
            speakers.insert(name.to_owned(), speak_config.build()?.into());
        }

        for (plugin_name, config) in &self.plugins {
            for instance in config.instances.iter().filter(|instance| {
                instance.type_ == PluginInstanceKind::Speaker
                    && self
                        .speakers
                        .as_ref()
                        .is_none_or(|v| v.contains(&instance.name))
            }) {
                if speakers.contains_key(&instance.name) {
                    return Err(Error::DuplicateInstance(format!(
                        "Multiple {:?} instances {:?} are found. Consider renaming one of the instances",
                        instance.type_, instance.name,
                    )));
                }

                speakers.insert(
                    instance.name.clone(),
                    Arc::new(instance.create_lazy_instance(
                        plugins,
                        plugin_name,
                        PluginProxy::new_speaker,
                    )?),
                );
            }
        }

        if self.speakers.is_none() && speakers.is_empty() {
            speakers.insert(
                Self::DEFAULT_SPEAKER_NAME.to_owned(),
                SpeakConfig::default().build()?.into(),
            );
        }
        Ok(speakers)
    }

    fn create_raw_joint_trajectory_clients(
        &self,
        plugins: &mut PluginMap,
    ) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, Error> {
        // True if the name is included in the list of clients to use
        // or if no list is specified.
        let is_used = |client_name| {
            self.joint_trajectory_clients
                .as_ref()
                .is_none_or(|v| v.contains(client_name))
        };
        let urdf_viz_clients_configs: Vec<_> = self
            .urdf_viz_clients_configs
            .iter()
            .filter(|c| is_used(&c.name))
            .cloned()
            .collect();
        #[cfg(feature = "ros")]
        let ros_clients_configs: Vec<_> = self
            .ros_clients_configs
            .iter()
            .filter(|c| is_used(&c.name))
            .collect();
        #[cfg(feature = "ros")]
        let ros_action_clients_configs: Vec<_> = self
            .ros_action_clients_configs
            .iter()
            .filter(|c| is_used(&c.name))
            .collect();

        let mut urdf_robot = None;
        #[cfg(not(feature = "ros"))]
        let use_urdf = !urdf_viz_clients_configs.is_empty();
        #[cfg(feature = "ros")]
        let use_urdf = !urdf_viz_clients_configs.is_empty()
            || !ros_clients_configs.is_empty()
            || !ros_action_clients_configs.is_empty();
        if use_urdf {
            if let Some(urdf_path) = self.openrr_clients_config.urdf_full_path() {
                urdf_robot = Some(urdf_rs::utils::read_urdf_or_xacro(urdf_path)?);
            }
        }

        let mut clients = arci_urdf_viz::create_joint_trajectory_clients_lazy(
            urdf_viz_clients_configs,
            urdf_robot.as_ref(),
        )?;
        #[cfg(feature = "ros")]
        {
            clients.extend(arci_ros::create_joint_trajectory_clients_lazy(
                ros_clients_configs,
                urdf_robot.as_ref(),
            )?);
            clients.extend(arci_ros::create_joint_trajectory_clients_lazy(
                ros_action_clients_configs,
                urdf_robot.as_ref(),
            )?);
        }

        for (plugin_name, config) in &self.plugins {
            for instance in config.instances.iter().filter(|instance| {
                instance.type_ == PluginInstanceKind::JointTrajectoryClient
                    && self
                        .joint_trajectory_clients
                        .as_ref()
                        .is_none_or(|v| v.contains(&instance.name))
            }) {
                if clients.contains_key(&instance.name) {
                    return Err(Error::DuplicateInstance(format!(
                        "Multiple {:?} instances {:?} are found. Consider renaming one of the instances",
                        instance.type_, instance.name,
                    )));
                }

                let client = instance.create_lazy_instance(
                    plugins,
                    plugin_name,
                    PluginProxy::new_joint_trajectory_client,
                )?;
                // If the `PluginProxy::new_joint_trajectory_client` returns
                // `Err` or `None`, `JointTrajectoryClient::joint_names` will
                // panic. Therefore, initialize it here to allow the user to
                // handle error.
                // `JointTrajectoryClientsContainer::new`, which is called inside
                // `RobotClient::new`, calls `JointTrajectoryClient::joint_names`,
                // so it makes no sense to make `JointTrajectoryClient` lazy here.
                client.get_ref()?;
                clients.insert(instance.name.clone(), Arc::new(client));
            }
        }

        Ok(clients)
    }
}

/// Convert relative path into absolute one
#[allow(clippy::needless_borrows_for_generic_args)] // https://github.com/rust-lang/rust-clippy/issues/13170
fn resolve_audio_file_path<P: AsRef<Path>>(
    base_path: P,
    relative_hash_map: &mut HashMap<String, PathBuf>,
) -> Result<(), Error> {
    for v in relative_hash_map.values_mut() {
        let full_path = openrr_client::resolve_relative_path(base_path.as_ref(), &v)?;
        *v = full_path;
    }
    Ok(())
}

fn instance_create_error<T: fmt::Debug, U>(
    res: Result<T, arci::Error>,
    instance_kind: PluginInstanceKind,
    instance_name: String,
    plugin_name: String,
) -> Result<U, arci::Error> {
    error!(
        "failed to create `{instance_kind:?}` instance `{instance_name}` from plugin `{plugin_name}`: {res:?}",
    );
    res.and_then(|_| {
        // TODO: error msg
        Err(format_err!(
            "failed to create `{instance_kind:?}` instance `{instance_name}` from plugin `{plugin_name}`: None",
        )
        .into())
    })
}

impl RobotConfig {
    pub fn new<P: AsRef<Path>>(path: P) -> Result<Self, Error> {
        Self::from_str(
            &std::fs::read_to_string(&path)
                .map_err(|e| Error::NoFile(path.as_ref().to_owned(), e))?,
            &path,
        )
    }

    pub fn from_str<P: AsRef<Path>>(s: &str, path: P) -> Result<Self, Error> {
        let path = path.as_ref();

        let mut config: RobotConfig =
            toml::from_str(s).map_err(|e| Error::TomlParseFailure(path.to_owned(), e))?;

        config.validate_ros_config()?;

        if config.openrr_clients_config.urdf_path.is_some() {
            config.openrr_clients_config.resolve_path(path)?;
        }
        for speak_config in config.speak_configs.values_mut() {
            if let SpeakConfig::Audio { map } = speak_config {
                resolve_audio_file_path(path, map)?;
            }
        }
        for plugin_config in config.plugins.values_mut() {
            resolve_plugin_path(&mut plugin_config.path, path)?;
            for instance in &mut plugin_config.instances {
                if let Some(args_path) = instance.args_from_path.take() {
                    instance.args_from_path =
                        Some(openrr_client::resolve_relative_path(path, args_path)?);
                }
            }
        }
        debug!("{config:?}");
        Ok(config)
    }

    /// Returns an error if a config requires ros feature but ros feature is disabled.
    fn validate_ros_config(&self) -> Result<(), Error> {
        if cfg!(feature = "ros") {
            return Ok(());
        }

        for (name, speak_config) in &self.speak_configs {
            if matches!(speak_config, SpeakConfig::RosEspeak { .. }) {
                return Err(Error::ConfigRequireRos(format!("speak_configs.{name}")));
            }
        }
        if !self.ros_clients_configs.is_empty() {
            return Err(Error::ConfigRequireRos("ros_clients_configs".into()));
        }
        if !self.ros_action_clients_configs.is_empty() {
            return Err(Error::ConfigRequireRos("ros_action_clients_configs".into()));
        }
        match self.move_base {
            ClientKind::Builtin(BuiltinClient::Ros) => {
                return Err(Error::ConfigRequireRos("move_base".into()));
            }
            ClientKind::Auto(true) if self.ros_cmd_vel_move_base_client_config.is_some() => {
                return Err(Error::ConfigRequireRos(
                    "ros_cmd_vel_move_base_client_config".into(),
                ));
            }
            _ => {}
        }
        match self.navigation {
            ClientKind::Builtin(BuiltinClient::Ros) => {
                return Err(Error::ConfigRequireRos("navigation".into()));
            }
            ClientKind::Auto(true) if self.ros_navigation_client_config.is_some() => {
                return Err(Error::ConfigRequireRos(
                    "ros_navigation_client_config".into(),
                ));
            }
            _ => {}
        }
        match self.localization {
            ClientKind::Builtin(BuiltinClient::Ros) => {
                return Err(Error::ConfigRequireRos("localization".into()));
            }
            ClientKind::Auto(true) if self.ros_localization_client_config.is_some() => {
                return Err(Error::ConfigRequireRos(
                    "ros_localization_client_config".into(),
                ));
            }
            _ => {}
        }
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_resolve_audio_file_path() {
        let mut hash = HashMap::new();
        hash.insert("a".to_owned(), PathBuf::from("dir1/file.mp3"));
        hash.insert("b".to_owned(), PathBuf::from("../dir2/file.mp3"));
        resolve_audio_file_path("/config/some_file.toml", &mut hash).unwrap();
        assert_eq!(hash.len(), 2);
        assert_eq!(hash["a"], PathBuf::from("/config/dir1/file.mp3"));
        assert_eq!(hash["b"], PathBuf::from("/config/../dir2/file.mp3"));
    }
}
