#[cfg(feature = "ros")]
use arci_ros::{RosControlClientConfig, RosEspeakClientConfig};
use arci_urdf_viz::UrdfVizWebClientConfig;
use openrr_client::{CollisionCheckClientConfig, IkClientConfig};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

use crate::Error;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct RobotConfig {
    pub urdf_path: String,
    urdf_full_path: Option<PathBuf>,
    pub self_collision_check_pairs: Vec<String>,

    #[cfg(feature = "ros")]
    pub ros_clients_configs: Vec<RosControlClientConfig>,
    pub urdf_viz_clients_configs: Vec<UrdfVizWebClientConfig>,

    #[cfg(feature = "ros")]
    pub ros_espeak_client_config: Option<RosEspeakClientConfig>,

    pub collision_check_clients_configs: Vec<CollisionCheckClientConfig>,
    pub ik_clients_configs: Vec<IkClientConfig>,
}

impl RobotConfig {
    pub fn try_new<P: AsRef<std::path::Path>>(path: P) -> Result<Self, Error> {
        let mut config: RobotConfig = toml::from_str(
            &std::fs::read_to_string(&path)
                .map_err(|e| Error::NoFile(path.as_ref().to_owned(), e))?,
        )
        .map_err(|e| Error::TomlParseFailure(path.as_ref().to_owned(), e))?;
        config.urdf_full_path = Some(
            path.as_ref()
                .parent()
                .ok_or_else(|| Error::NoParentDirectory(path.as_ref().to_owned()))?
                .join(&config.urdf_path),
        );
        Ok(config)
    }
    pub fn urdf_full_path(&self) -> &Option<PathBuf> {
        &self.urdf_full_path
    }
}
