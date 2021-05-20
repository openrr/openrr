use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};

use arci::{JointTrajectoryClient, Localization, MoveBase, Navigation, Speaker};
#[cfg(feature = "ros")]
use arci_ros::{
    RosCmdVelMoveBase, RosCmdVelMoveBaseConfig, RosControlClientConfig, RosEspeakClient,
    RosEspeakClientConfig, RosLocalizationClient, RosLocalizationClientConfig, RosNavClient,
    RosNavClientConfig,
};
use arci_speak_audio::AudioSpeaker;
use arci_speak_cmd::LocalCommand;
use arci_urdf_viz::{create_joint_trajectory_clients, UrdfVizWebClient, UrdfVizWebClientConfig};
use openrr_client::{OpenrrClientsConfig, PrintSpeaker, RobotClient};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::debug;

use crate::Error;

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(tag = "type", content = "args")]
#[serde(deny_unknown_fields)]
#[non_exhaustive] // The variants will increase depending on the feature flag.
pub enum SpeakConfig {
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
    #[serde(rename = "RosEspeak")]
    __RosEspeak {
        #[schemars(schema_with = "unimplemented_schema")]
        config: toml::Value,
    },
    Audio {
        map: HashMap<String, PathBuf>,
    },
}

impl Default for SpeakConfig {
    fn default() -> Self {
        SpeakConfig::Print
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
#[non_exhaustive] // The fields will increase depending on the feature flag.
pub struct RobotConfig {
    // TOML format has a restriction that if a table itself contains tables,
    // all keys with non-table values must be emitted first.
    // Therefore, these fields must be located at the start of the struct.
    #[serde(default = "default_urdf_viz_clients_total_complete_allowable_error")]
    pub urdf_viz_clients_total_complete_allowable_error: f64,
    #[serde(default = "default_urdf_viz_clients_complete_timeout_sec")]
    pub urdf_viz_clients_complete_timeout_sec: f64,
    #[serde(default = "default_true")]
    pub use_move_base_urdf_viz_web_client: bool,
    #[serde(default = "default_true")]
    pub use_navigation_urdf_viz_web_client: bool,
    #[serde(default = "default_true")]
    pub use_localization_urdf_viz_web_client: bool,

    #[cfg(feature = "ros")]
    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub ros_clients_configs: Vec<RosControlClientConfig>,
    // A dummy field to catch that there is a config that requires the ros feature.
    #[cfg(not(feature = "ros"))]
    #[schemars(schema_with = "unimplemented_schema")]
    ros_clients_configs: Option<toml::Value>,
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

    pub openrr_clients_config: OpenrrClientsConfig,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            ros_clients_configs: Default::default(),
            urdf_viz_clients_configs: Default::default(),
            urdf_viz_clients_total_complete_allowable_error:
                default_urdf_viz_clients_total_complete_allowable_error(),
            urdf_viz_clients_complete_timeout_sec: default_urdf_viz_clients_complete_timeout_sec(),
            speak_configs: Default::default(),
            ros_cmd_vel_move_base_client_config: Default::default(),
            use_move_base_urdf_viz_web_client: default_true(),
            ros_navigation_client_config: Default::default(),
            use_navigation_urdf_viz_web_client: default_true(),
            ros_localization_client_config: Default::default(),
            use_localization_urdf_viz_web_client: default_true(),
            openrr_clients_config: Default::default(),
        }
    }
}

fn default_urdf_viz_clients_total_complete_allowable_error() -> f64 {
    0.02
}

fn default_urdf_viz_clients_complete_timeout_sec() -> f64 {
    3.0
}

fn default_true() -> bool {
    true
}

// Creates dummy schema for dummy fields.
#[cfg(not(feature = "ros"))]
fn unimplemented_schema(_gen: &mut schemars::gen::SchemaGenerator) -> schemars::schema::Schema {
    unimplemented!()
}

impl RobotConfig {
    const DEFAULT_SPEAKER_NAME: &'static str = "Default";

    #[cfg(feature = "ros")]
    pub fn has_ros_clients(&self) -> bool {
        let mut has_ros_espeak = false;
        let speak_configs = self.speak_configs.clone();
        for (_, speak_config) in speak_configs {
            has_ros_espeak |= matches!(speak_config, SpeakConfig::RosEspeak { config: _ });
        }
        !self.ros_clients_configs.is_empty()
            || has_ros_espeak
            || self.ros_cmd_vel_move_base_client_config.is_some()
            || self.ros_navigation_client_config.is_some()
    }

    pub fn create_robot_client<L, M, N>(&self) -> Result<RobotClient<L, M, N>, Error>
    where
        L: Localization + From<Box<dyn Localization>>,
        M: MoveBase + From<Box<dyn MoveBase>>,
        N: Navigation + From<Box<dyn Navigation>>,
    {
        let mut speakers = HashMap::new();
        for (name, speaker) in self.create_speakers() {
            speakers.insert(name, speaker.into());
        }

        Ok(RobotClient::try_new(
            self.openrr_clients_config.clone(),
            self.create_raw_joint_trajectory_clients()?,
            speakers,
            self.create_localization().map(|l| l.into()),
            self.create_move_base().map(|m| m.into()),
            self.create_navigation().map(|n| n.into()),
        )?)
    }

    fn create_localization_without_ros(&self) -> Option<Box<dyn Localization>> {
        if self.use_localization_urdf_viz_web_client {
            let urdf_viz_client = Box::new(UrdfVizWebClient::default());
            Some(urdf_viz_client as Box<dyn Localization>)
        } else {
            None
        }
    }

    #[cfg(feature = "ros")]
    fn create_localization_with_ros(&self) -> Option<Box<dyn Localization>> {
        if let Some(ros_localization_client_config) = &self.ros_localization_client_config {
            Some(Box::new(RosLocalizationClient::new_from_config(
                ros_localization_client_config.clone(),
            )) as Box<dyn Localization>)
        } else {
            self.create_localization_without_ros()
        }
    }

    fn create_localization(&self) -> Option<Box<dyn Localization>> {
        #[cfg(not(feature = "ros"))]
        {
            self.create_localization_without_ros()
        }
        #[cfg(feature = "ros")]
        {
            self.create_localization_with_ros()
        }
    }

    fn create_navigation_without_ros(&self) -> Option<Box<dyn Navigation>> {
        if self.use_navigation_urdf_viz_web_client {
            let urdf_viz_client = Box::new(UrdfVizWebClient::default());
            Some(urdf_viz_client as Box<dyn Navigation>)
        } else {
            None
        }
    }

    #[cfg(feature = "ros")]
    fn create_navigation_with_ros(&self) -> Option<Box<dyn Navigation>> {
        if let Some(ros_navigation_client_config) = &self.ros_navigation_client_config {
            Some(Box::new(RosNavClient::new_from_config(
                ros_navigation_client_config.clone(),
            )) as Box<dyn Navigation>)
        } else {
            self.create_navigation_without_ros()
        }
    }

    fn create_navigation(&self) -> Option<Box<dyn Navigation>> {
        #[cfg(not(feature = "ros"))]
        {
            self.create_navigation_without_ros()
        }
        #[cfg(feature = "ros")]
        {
            self.create_navigation_with_ros()
        }
    }

    fn create_move_base_without_ros(&self) -> Option<Box<dyn MoveBase>> {
        if self.use_move_base_urdf_viz_web_client {
            let urdf_viz_client = Box::new(UrdfVizWebClient::default());
            urdf_viz_client.run_thread();
            Some(urdf_viz_client as Box<dyn MoveBase>)
        } else {
            None
        }
    }

    #[cfg(feature = "ros")]
    fn create_move_base_with_ros(&self) -> Option<Box<dyn MoveBase>> {
        if let Some(ros_cmd_vel_move_base_client_config) = &self.ros_cmd_vel_move_base_client_config
        {
            Some(Box::new(RosCmdVelMoveBase::new(
                &ros_cmd_vel_move_base_client_config.topic,
            )) as Box<dyn MoveBase>)
        } else {
            self.create_move_base_without_ros()
        }
    }

    fn create_move_base(&self) -> Option<Box<dyn MoveBase>> {
        #[cfg(not(feature = "ros"))]
        {
            self.create_move_base_without_ros()
        }
        #[cfg(feature = "ros")]
        {
            self.create_move_base_with_ros()
        }
    }

    fn create_print_speaker(&self) -> Box<dyn Speaker> {
        Box::new(PrintSpeaker::new())
    }

    fn create_local_command_speaker(&self) -> Box<dyn Speaker> {
        Box::new(LocalCommand::new())
    }

    fn create_audio_speaker(&self, hash_map: HashMap<String, PathBuf>) -> Box<dyn Speaker> {
        Box::new(AudioSpeaker::new(hash_map))
    }

    #[cfg(feature = "ros")]
    fn create_ros_espeak_client(&self, topic: &str) -> Box<dyn Speaker> {
        Box::new(RosEspeakClient::new(topic))
    }

    fn create_speakers(&self) -> HashMap<String, Box<dyn Speaker>> {
        let mut speakers = HashMap::new();
        for (name, speak_config) in &self.speak_configs {
            speakers.insert(
                name.to_owned(),
                match speak_config {
                    #[cfg(feature = "ros")]
                    SpeakConfig::RosEspeak { config } => {
                        self.create_ros_espeak_client(&config.topic)
                    }
                    #[cfg(not(feature = "ros"))]
                    SpeakConfig::__RosEspeak { .. } => unreachable!(),
                    SpeakConfig::Audio { ref map } => self.create_audio_speaker(map.clone()),
                    SpeakConfig::Command => self.create_local_command_speaker(),
                    SpeakConfig::Print => self.create_print_speaker(),
                },
            );
        }
        if speakers.is_empty() {
            speakers.insert(
                Self::DEFAULT_SPEAKER_NAME.to_owned(),
                self.create_print_speaker(),
            );
        }
        speakers
    }

    fn create_raw_joint_trajectory_clients(
        &self,
    ) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, Error> {
        let urdf_robot = if let Some(urdf_path) = self.openrr_clients_config.urdf_full_path() {
            let urdf_robot = urdf_rs::utils::read_urdf_or_xacro(urdf_path)?;
            Some(urdf_robot)
        } else {
            None
        };

        #[cfg(not(feature = "ros"))]
        let raw_joint_trajectory_clients = create_joint_trajectory_clients(
            self.urdf_viz_clients_configs.clone(),
            self.urdf_viz_clients_total_complete_allowable_error,
            self.urdf_viz_clients_complete_timeout_sec,
            urdf_robot.as_ref(),
        )?;
        #[cfg(feature = "ros")]
        let raw_joint_trajectory_clients = {
            let mut clients = if self.urdf_viz_clients_configs.is_empty() {
                HashMap::new()
            } else {
                create_joint_trajectory_clients(
                    self.urdf_viz_clients_configs.clone(),
                    self.urdf_viz_clients_total_complete_allowable_error,
                    self.urdf_viz_clients_complete_timeout_sec,
                    urdf_robot.as_ref(),
                )?
            };
            clients.extend(
                arci_ros::create_joint_trajectory_clients(
                    self.ros_clients_configs.clone(),
                    urdf_robot.as_ref(),
                )?
                .into_iter(),
            );
            clients
        };
        Ok(raw_joint_trajectory_clients)
    }
}

/// Convert relative path into absolute one
fn resolve_audio_file_path<P: AsRef<Path>>(
    base_path: P,
    relative_hash_map: &mut HashMap<String, PathBuf>,
) -> Result<(), Error> {
    for v in relative_hash_map.values_mut() {
        let full_path =
            openrr_client::resolve_relative_path(base_path.as_ref().to_owned(), v.to_owned())?;
        *v = full_path;
    }
    Ok(())
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

impl RobotConfig {
    pub fn try_new<P: AsRef<Path>>(path: P) -> Result<Self, Error> {
        Self::from_str(
            &std::fs::read_to_string(&path)
                .map_err(|e| Error::NoFile(path.as_ref().to_owned(), e))?,
            &path,
        )
    }

    pub fn from_str<P: AsRef<Path>>(s: &str, path: P) -> Result<Self, Error> {
        let mut config: RobotConfig =
            toml::from_str(s).map_err(|e| Error::TomlParseFailure(path.as_ref().to_owned(), e))?;

        // Returns an error if a config requires ros feature but ros feature is disabled.
        #[cfg(not(feature = "ros"))]
        {
            for (name, speak_config) in &config.speak_configs {
                if matches!(speak_config, SpeakConfig::__RosEspeak { .. }) {
                    return Err(Error::ConfigRequireRos(format!("speak_configs.{}", name)));
                }
            }
            if config.ros_clients_configs.is_some() {
                return Err(Error::ConfigRequireRos("ros_clients_configs".into()));
            }
            if config.ros_cmd_vel_move_base_client_config.is_some() {
                return Err(Error::ConfigRequireRos(
                    "ros_cmd_vel_move_base_client_config".into(),
                ));
            }
            if config.ros_navigation_client_config.is_some() {
                return Err(Error::ConfigRequireRos(
                    "ros_navigation_client_config".into(),
                ));
            }
        }

        if config.openrr_clients_config.urdf_path.is_some() {
            config.openrr_clients_config.resolve_path(path.as_ref())?;
        }
        for speak_config in config.speak_configs.values_mut() {
            if let SpeakConfig::Audio { ref mut map } = speak_config {
                resolve_audio_file_path(path.as_ref(), map)?;
            };
        }
        debug!("{:?}", config);
        Ok(config)
    }
}
