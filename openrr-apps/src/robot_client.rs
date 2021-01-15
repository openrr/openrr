use arci::{JointTrajectoryClient, Speaker};
use arci_urdf_viz::create_joint_trajectory_clients;
use k::{Chain, Isometry3};
use openrr_client::PrintSpeaker;
use openrr_client::{
    create_collision_check_clients, create_ik_clients, CollisionCheckClient, IkClient,
};
use std::{collections::HashMap, sync::Arc, time::Duration};

use crate::{Error, RobotConfig};
#[cfg(feature = "ros")]
use arci_ros::RosEspeakClient;

type ArcIkClient = Arc<IkClient<Arc<dyn JointTrajectoryClient>>>;

pub struct RobotClient {
    full_chain_for_collision_checker: Arc<Chain<f64>>,
    raw_joint_trajectory_clients: HashMap<String, Arc<dyn JointTrajectoryClient>>,
    all_joint_trajectory_clients: HashMap<String, Arc<dyn JointTrajectoryClient>>,
    collision_check_clients:
        HashMap<String, Arc<CollisionCheckClient<Arc<dyn JointTrajectoryClient>>>>,
    ik_clients: HashMap<String, ArcIkClient>,
    speaker: Arc<dyn Speaker>,
    joints_poses: HashMap<String, HashMap<String, Vec<f64>>>,
}

impl RobotClient {
    pub fn try_new(config: RobotConfig) -> Result<Self, Error> {
        #[cfg(not(feature = "ros"))]
        let raw_joint_trajectory_clients = if config.urdf_viz_clients_configs.is_empty() {
            return Err(Error::NoClientsConfigs("urdf_viz_clients".to_owned()));
        } else {
            create_joint_trajectory_clients(config.urdf_viz_clients_configs.clone())
        };
        #[cfg(feature = "ros")]
        let raw_joint_trajectory_clients = {
            let mut clients = if config.urdf_viz_clients_configs.is_empty() {
                HashMap::new()
            } else {
                create_joint_trajectory_clients(config.urdf_viz_clients_configs.clone())
            };
            clients.extend(
                arci_ros::create_joint_trajectory_clients(config.ros_clients_configs.clone())
                    .into_iter(),
            );
            if clients.is_empty() {
                return Err(Error::NoClientsConfigs(
                    "urdf_viz_clients_configs / ros_clients_configs".to_owned(),
                ));
            }
            clients
        };

        #[cfg(not(feature = "ros"))]
        let speaker: Arc<dyn Speaker> = Arc::new(PrintSpeaker::new());
        #[cfg(feature = "ros")]
        let speaker: Arc<dyn Speaker> = if let Some(c) = &config.ros_espeak_client_config {
            Arc::new(RosEspeakClient::new(&c.topic))
        } else {
            Arc::new(PrintSpeaker::new())
        };

        let urdf_full_path = if let Some(p) = config.urdf_full_path().clone() {
            p
        } else {
            return Err(Error::NoUrdfPath);
        };
        let full_chain_for_collision_checker = Arc::new(Chain::from_urdf_file(&urdf_full_path)?);

        let collision_check_clients = create_collision_check_clients(
            urdf_full_path,
            &config.self_collision_check_pairs,
            &config.collision_check_clients_configs,
            &raw_joint_trajectory_clients,
            full_chain_for_collision_checker.clone(),
        );

        let mut all_joint_trajectory_clients = HashMap::new();
        for (name, client) in &raw_joint_trajectory_clients {
            all_joint_trajectory_clients.insert(name.to_owned(), client.clone());
        }
        for (name, client) in &collision_check_clients {
            all_joint_trajectory_clients.insert(name.to_owned(), client.clone());
        }

        let ik_clients = create_ik_clients(
            &config.ik_clients_configs,
            &all_joint_trajectory_clients,
            &full_chain_for_collision_checker,
        );

        let mut joints_poses: HashMap<String, HashMap<String, Vec<f64>>> = HashMap::new();
        for joints_pose in &config.joints_poses {
            joints_poses
                .entry(joints_pose.client_name.clone())
                .or_insert_with(HashMap::new)
                .insert(
                    joints_pose.pose_name.to_owned(),
                    joints_pose.positions.to_owned(),
                );
        }
        Ok(Self {
            full_chain_for_collision_checker,
            raw_joint_trajectory_clients,
            all_joint_trajectory_clients,
            collision_check_clients,
            ik_clients,
            speaker,
            joints_poses,
        })
    }
    pub fn set_raw_clients_joint_positions_to_full_chain_for_collision_checker(
        &self,
    ) -> Result<(), Error> {
        for client in self.raw_joint_trajectory_clients.values() {
            let positions = client.current_joint_positions()?;
            let joint_names = client.joint_names();
            if positions.len() != joint_names.len() {
                return Err(Error::MismatchedLength(positions.len(), joint_names.len()));
            }
            for (index, joint_name) in joint_names.iter().enumerate() {
                if let Some(joint) = self.full_chain_for_collision_checker.find(joint_name) {
                    joint.set_joint_position_clamped(positions[index])
                } else {
                    return Err(Error::NoJoint(joint_name.to_owned()));
                }
            }
        }
        self.full_chain_for_collision_checker.update_transforms();
        Ok(())
    }

    pub fn is_raw_joint_trajectory_client(&self, name: &str) -> bool {
        self.raw_joint_trajectory_clients.contains_key(name)
    }
    pub fn is_joint_trajectory_client(&self, name: &str) -> bool {
        self.all_joint_trajectory_clients.contains_key(name)
    }
    pub fn is_collision_check_client(&self, name: &str) -> bool {
        self.collision_check_clients.contains_key(name)
    }
    pub fn is_ik_client(&self, name: &str) -> bool {
        self.ik_clients.contains_key(name)
    }

    fn joint_trajectory_client(
        &self,
        name: &str,
    ) -> Result<&Arc<dyn JointTrajectoryClient>, Error> {
        if self.is_joint_trajectory_client(name) {
            Ok(&self.all_joint_trajectory_clients[name])
        } else {
            Err(Error::NoJointTrajectoryClient(name.to_owned()))
        }
    }
    fn ik_client(&self, name: &str) -> Result<&ArcIkClient, Error> {
        if self.is_ik_client(name) {
            Ok(&self.ik_clients[name])
        } else {
            Err(Error::NoIkClient(name.to_owned()))
        }
    }

    pub async fn send_joint_positions(
        &self,
        name: &str,
        positions: &[f64],
        duration_sec: f64,
    ) -> Result<(), Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        if self.is_ik_client(name) {
            Ok(self
                .ik_client(name)?
                .client
                .send_joint_positions(positions.to_owned(), Duration::from_secs_f64(duration_sec))
                .await?)
        } else {
            Ok(self
                .joint_trajectory_client(name)?
                .send_joint_positions(positions.to_owned(), Duration::from_secs_f64(duration_sec))
                .await?)
        }
    }
    pub async fn current_joint_positions(&self, name: &str) -> Result<Vec<f64>, Error> {
        if self.is_ik_client(name) {
            self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
            Ok(self.ik_client(name)?.current_joint_positions()?)
        } else {
            Ok(self
                .joint_trajectory_client(name)?
                .current_joint_positions()?)
        }
    }
    pub async fn send_joints_pose(
        &self,
        name: &str,
        pose_name: &str,
        duration_sec: f64,
    ) -> Result<(), Error> {
        if self.joints_poses.contains_key(name) && self.joints_poses[name].contains_key(pose_name) {
            Ok(self
                .send_joint_positions(name, &self.joints_poses[name][pose_name], duration_sec)
                .await?)
        } else {
            Err(Error::NoJointsPose(name.to_owned(), pose_name.to_owned()))
        }
    }
    pub async fn current_end_transform(&self, name: &str) -> Result<Isometry3<f64>, Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        Ok(self.ik_client(name)?.current_end_transform()?)
    }
    pub async fn transform(
        &self,
        name: &str,
        pose: &Isometry3<f64>,
    ) -> Result<Isometry3<f64>, Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        Ok(self.ik_client(name)?.transform(pose)?)
    }
    pub async fn move_ik(
        &self,
        name: &str,
        target_pose: &Isometry3<f64>,
        duration_sec: f64,
    ) -> Result<(), Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        Ok(self
            .ik_client(name)?
            .move_ik(target_pose, duration_sec)
            .await?)
    }
    pub async fn move_ik_with_interpolation(
        &self,
        name: &str,
        target_pose: &Isometry3<f64>,
        duration_sec: f64,
    ) -> Result<(), Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        Ok(self
            .ik_client(name)?
            .move_ik_with_interpolation(target_pose, duration_sec)
            .await?)
    }
    pub async fn send_joint_positions_with_pose_interpolation(
        &self,
        name: &str,
        positions: &[f64],
        duration_sec: f64,
    ) -> Result<(), Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        let target_pose = {
            let ik_client = self.ik_client(name)?;
            ik_client.chain.set_joint_positions_clamped(positions);
            ik_client.ik_solver_with_chain.end_transform()
        };
        Ok(self
            .move_ik_with_interpolation(name, &target_pose, duration_sec)
            .await?)
    }

    pub fn raw_joint_trajectory_clients_names(&self) -> Vec<String> {
        self.raw_joint_trajectory_clients
            .keys()
            .map(|k| k.to_owned())
            .collect::<Vec<String>>()
    }
    pub fn joint_trajectory_clients_names(&self) -> Vec<String> {
        self.all_joint_trajectory_clients
            .keys()
            .map(|k| k.to_owned())
            .collect::<Vec<String>>()
    }
    pub fn collision_check_clients_names(&self) -> Vec<String> {
        self.collision_check_clients
            .keys()
            .map(|k| k.to_owned())
            .collect::<Vec<String>>()
    }
    pub fn ik_clients_names(&self) -> Vec<String> {
        self.ik_clients
            .keys()
            .map(|k| k.to_owned())
            .collect::<Vec<String>>()
    }

    pub fn speak(&self, message: &str) {
        self.speaker.speak(message)
    }
}
