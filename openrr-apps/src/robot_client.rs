use arci::JointTrajectoryClient;
use arci_urdf_viz::create_joint_trajectory_clients;
use k::{Chain, Isometry3};
use log::info;
use openrr_client::{
    create_collision_check_clients, create_ik_clients, CollisionCheckClient, IkClient,
};
use std::{collections::HashMap, sync::Arc, time::Duration};
use tokio::sync::Mutex;

use crate::{Error, RobotConfig};

type ArcMutexIkClient = Arc<Mutex<IkClient<Arc<dyn JointTrajectoryClient>>>>;

pub struct RobotClient {
    pub full_chain_for_collision_checker: Arc<Chain<f64>>,
    pub config: RobotConfig,
    pub raw_joint_trajectory_clients: HashMap<String, Arc<dyn JointTrajectoryClient>>,
    pub all_joint_trajectory_clients: HashMap<String, Arc<dyn JointTrajectoryClient>>,
    pub collision_check_clients:
        HashMap<String, Arc<CollisionCheckClient<Arc<dyn JointTrajectoryClient>>>>,
    pub ik_clients: HashMap<String, ArcMutexIkClient>,
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
        Ok(Self {
            full_chain_for_collision_checker,
            config,
            raw_joint_trajectory_clients,
            all_joint_trajectory_clients,
            collision_check_clients,
            ik_clients,
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
    fn ik_client(&self, name: &str) -> Result<&ArcMutexIkClient, Error> {
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
        Ok(self
            .joint_trajectory_client(name)?
            .send_joint_positions(positions.to_owned(), Duration::from_secs_f64(duration_sec))
            .await?)
    }
    pub fn current_joint_positions(&self, name: &str) -> Result<Vec<f64>, Error> {
        Ok(self
            .joint_trajectory_client(name)?
            .current_joint_positions()?)
    }
    pub async fn current_end_transform(&self, name: &str) -> Result<k::Isometry3<f64>, Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        Ok(self.ik_client(name)?.lock().await.current_end_transform()?)
    }
    pub async fn ik_client_current_joint_positions(&self, name: &str) -> Result<Vec<f64>, Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        Ok(self
            .ik_client(name)?
            .lock()
            .await
            .current_joint_positions()?)
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
            .lock()
            .await
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
            .lock()
            .await
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
        let ik_client = self.ik_client(name)?.lock().await;
        ik_client.chain.set_joint_positions_clamped(positions);
        let target_pose = ik_client.ik_solver_with_chain.end_transform();
        Ok(self
            .move_ik_with_interpolation(name, &target_pose, duration_sec)
            .await?)
    }

    pub fn list_clients(&self) {
        info!("Raw joint trajectory clients");
        for name in self.raw_joint_trajectory_clients.keys() {
            info!(" {}", name);
        }
        info!("Joint trajectory clients");
        for name in self.all_joint_trajectory_clients.keys() {
            info!(" {}", name);
        }
        info!("Collision check clients");
        for name in self.collision_check_clients.keys() {
            info!(" {}", name);
        }
        info!("Ik clients");
        for name in self.ik_clients.keys() {
            info!(" {}", name);
        }
    }
}
