use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};

use anyhow::Result;
use arci::{JointTrajectoryClient, Localization, MoveBase, Navigation, Speaker};
use arci_urdf_viz::{create_joint_trajectory_clients, UrdfVizWebClient, UrdfVizWebClientConfig};
use openrr_client::{OpenrrClientsConfig, PrintSpeaker, RobotClient};
use serde::{Deserialize, Serialize};
use tracing::debug;

#[derive(Debug, Serialize, Deserialize, Clone)]
#[non_exhaustive] // The fields will increase depending on the feature flag.
pub struct RobotConfig {
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

    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub urdf_viz_clients_configs: Vec<UrdfVizWebClientConfig>,

    pub openrr_clients_config: OpenrrClientsConfig,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            urdf_viz_clients_configs: Default::default(),
            urdf_viz_clients_total_complete_allowable_error:
                default_urdf_viz_clients_total_complete_allowable_error(),
            urdf_viz_clients_complete_timeout_sec: default_urdf_viz_clients_complete_timeout_sec(),
            use_move_base_urdf_viz_web_client: default_true(),
            use_navigation_urdf_viz_web_client: default_true(),
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

impl RobotConfig {
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

    pub fn create_robot_client<L, M, N>(&self) -> Result<RobotClient<L, M, N>>
    where
        L: Localization + From<Box<dyn Localization>>,
        M: MoveBase + From<Box<dyn MoveBase>>,
        N: Navigation + From<Box<dyn Navigation>>,
    {
        Ok(RobotClient::try_new(
            self.openrr_clients_config.clone(),
            self.create_raw_joint_trajectory_clients(),
            Default::default(),
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

impl RobotConfig {
    pub fn try_new<P: AsRef<Path>>(path: P) -> Result<Self> {
        let mut config: RobotConfig = toml::from_str(&std::fs::read_to_string(&path)?)?;

        if config.openrr_clients_config.urdf_path.is_some() {
            config.openrr_clients_config.resolve_path(path.as_ref())?;
        }
        debug!("{:?}", config);
        Ok(config)
    }
}
