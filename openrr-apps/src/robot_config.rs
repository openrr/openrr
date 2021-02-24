use crate::Error;
use arci::{JointTrajectoryClient, MoveBase, Navigation, Speaker};
#[cfg(feature = "ros")]
use arci_ros::{
    RosCmdVelMoveBase, RosCmdVelMoveBaseConfig, RosControlClientConfig, RosEspeakClient,
    RosEspeakClientConfig, RosNavClient, RosNavClientConfig,
};
use arci_urdf_viz::{create_joint_trajectory_clients, UrdfVizWebClient, UrdfVizWebClientConfig};

use arci_speak_audio::AudioSpeaker;
use arci_speak_cmd::LocalCommand;
use openrr_client::{OpenrrClientsConfig, PrintSpeaker, RobotClient};
use serde::{Deserialize, Serialize};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};
use tracing::debug;

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "type", content = "args")]
pub enum SpeakConfig {
    Print,
    Command,
    #[cfg(feature = "ros")]
    RosEspeak {
        config: RosEspeakClientConfig,
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

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct RobotConfig {
    #[cfg(feature = "ros")]
    #[cfg_attr(feature = "ros", serde(default))]
    pub ros_clients_configs: Vec<RosControlClientConfig>,
    #[serde(default)]
    pub urdf_viz_clients_configs: Vec<UrdfVizWebClientConfig>,

    #[serde(default = "default_urdf_viz_clients_total_complete_allowable_error")]
    pub urdf_viz_clients_total_complete_allowable_error: f64,
    #[serde(default = "default_urdf_viz_clients_complete_timeout_sec")]
    pub urdf_viz_clients_complete_timeout_sec: f64,

    #[serde(default)]
    pub speak_config: SpeakConfig,

    #[cfg(feature = "ros")]
    pub ros_cmd_vel_move_base_client_config: Option<RosCmdVelMoveBaseConfig>,
    #[serde(default = "default_true")]
    pub use_move_base_urdf_viz_web_client: bool,

    #[cfg(feature = "ros")]
    pub ros_navigation_client_config: Option<RosNavClientConfig>,
    #[serde(default = "default_true")]
    pub use_navigation_urdf_viz_web_client: bool,

    pub openrr_clients_config: OpenrrClientsConfig,
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

impl RobotConfig {
    #[cfg(feature = "ros")]
    pub fn has_ros_clients(&self) -> bool {
        !self.ros_clients_configs.is_empty()
            || matches!(&self.speak_config, SpeakConfig::RosEspeak { config: _ })
            || self.ros_cmd_vel_move_base_client_config.is_some()
            || self.ros_navigation_client_config.is_some()
    }

    pub fn create_robot_client<S, M, N>(&self) -> Result<RobotClient<S, M, N>, Error>
    where
        S: Speaker + From<Box<dyn Speaker>>,
        M: MoveBase + From<Box<dyn MoveBase>>,
        N: Navigation + From<Box<dyn Navigation>>,
    {
        Ok(RobotClient::try_new(
            self.openrr_clients_config.clone(),
            self.create_raw_joint_trajectory_clients(),
            self.create_speaker().into(),
            self.create_move_base().map(|m| m.into()),
            self.create_navigation().map(|n| n.into()),
        )?)
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
    fn create_speaker_print_speaker(&self) -> Box<dyn Speaker> {
        Box::new(PrintSpeaker::new())
    }
    fn create_speaker_local_command(&self) -> Box<dyn Speaker> {
        Box::new(LocalCommand::new())
    }
    fn create_speaker_audio_speaker(&self, hash_map: HashMap<String, PathBuf>) -> Box<dyn Speaker> {
        Box::new(AudioSpeaker::new(hash_map))
    }
    #[cfg(feature = "ros")]
    fn create_speaker_ros_espeak_client(&self, topic: &str) -> Box<dyn Speaker> {
        Box::new(RosEspeakClient::new(topic))
    }
    fn create_speaker(&self) -> Box<dyn Speaker> {
        match &self.speak_config {
            #[cfg(feature = "ros")]
            SpeakConfig::RosEspeak { config } => {
                self.create_speaker_ros_espeak_client(&config.topic)
            }
            SpeakConfig::Audio { ref map } => self.create_speaker_audio_speaker(map.clone()),
            SpeakConfig::Command => self.create_speaker_local_command(),
            SpeakConfig::Print => self.create_speaker_print_speaker(),
        }
    }
    fn create_raw_joint_trajectory_clients(
        &self,
    ) -> HashMap<String, Arc<dyn JointTrajectoryClient>> {
        #[cfg(not(feature = "ros"))]
        let raw_joint_trajectory_clients = create_joint_trajectory_clients(
            self.urdf_viz_clients_configs.clone(),
            self.urdf_viz_clients_total_complete_allowable_error,
            self.urdf_viz_clients_complete_timeout_sec,
        );
        #[cfg(feature = "ros")]
        let raw_joint_trajectory_clients = {
            let mut clients = if self.urdf_viz_clients_configs.is_empty() {
                HashMap::new()
            } else {
                create_joint_trajectory_clients(
                    self.urdf_viz_clients_configs.clone(),
                    self.urdf_viz_clients_total_complete_allowable_error,
                    self.urdf_viz_clients_complete_timeout_sec,
                )
            };
            clients.extend(
                arci_ros::create_joint_trajectory_clients(self.ros_clients_configs.clone())
                    .into_iter(),
            );
            clients
        };
        raw_joint_trajectory_clients
    }
}

/// Convert relative path into absolute one
fn resolve_audio_file_path<P: AsRef<Path>>(
    base_path: P,
    relative_hash_map: HashMap<String, PathBuf>,
) -> Result<HashMap<String, PathBuf>, Error> {
    let mut absolute_hash_map = HashMap::new();
    for (k, v) in relative_hash_map.iter() {
        let path_string = v.clone().into_os_string().into_string().unwrap();
        let full_path =
            openrr_client::resolve_relative_path(base_path.as_ref().to_owned(), &path_string)?
                .unwrap();
        absolute_hash_map.insert(k.clone(), full_path);
    }
    Ok(absolute_hash_map)
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_resolve_audio_file_path() {
        let mut hash = HashMap::new();
        hash.insert("a".to_owned(), PathBuf::from("dir1/file.mp3"));
        hash.insert("b".to_owned(), PathBuf::from("../dir2/file.mp3"));
        let resolved = resolve_audio_file_path("/config/some_file.toml", hash).unwrap();
        assert_eq!(resolved.len(), 2);
        assert_eq!(resolved["a"], PathBuf::from("/config/dir1/file.mp3"));
        assert_eq!(resolved["b"], PathBuf::from("/config/../dir2/file.mp3"));
    }
}

impl RobotConfig {
    pub fn try_new<P: AsRef<Path>>(path: P) -> Result<Self, Error> {
        let mut config: RobotConfig = toml::from_str(
            &std::fs::read_to_string(&path)
                .map_err(|e| Error::NoFile(path.as_ref().to_owned(), e))?,
        )
        .map_err(|e| Error::TomlParseFailure(path.as_ref().to_owned(), e))?;
        if config.openrr_clients_config.urdf_path.is_some() {
            config.openrr_clients_config.resolve_path(path.as_ref())?;
        }
        if let SpeakConfig::Audio { ref mut map } = config.speak_config {
            *map = resolve_audio_file_path(path, map.clone())?;
        };
        debug!("{:?}", config);
        Ok(config)
    }
}
