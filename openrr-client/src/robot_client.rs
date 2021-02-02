use crate::{
    create_collision_check_clients, create_ik_clients, CollisionCheckClient,
    CollisionCheckClientConfig, Error, IkClient, IkClientConfig, IkSolverWithChain,
};
use arci::{
    BaseVelocity, Error as ArciError, JointTrajectoryClient, JointTrajectoryClientsContainer,
    MoveBase, Navigation, Speaker,
};
use async_trait::async_trait;
use k::{nalgebra::Isometry2, Chain, Isometry3};
use log::debug;
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, path::Path, path::PathBuf, sync::Arc, time::Duration};

type ArcIkClient = Arc<IkClient<Arc<dyn JointTrajectoryClient>>>;
pub type ArcRobotClient = RobotClient<Arc<dyn Speaker>, Arc<dyn MoveBase>, Arc<dyn Navigation>>;
pub type BoxRobotClient = RobotClient<Box<dyn Speaker>, Box<dyn MoveBase>, Box<dyn Navigation>>;

pub struct RobotClient<S, M, N>
where
    S: Speaker,
    M: MoveBase,
    N: Navigation,
{
    full_chain_for_collision_checker: Arc<Chain<f64>>,
    raw_joint_trajectory_clients: HashMap<String, Arc<dyn JointTrajectoryClient>>,
    all_joint_trajectory_clients: HashMap<String, Arc<dyn JointTrajectoryClient>>,
    collision_check_clients:
        HashMap<String, Arc<CollisionCheckClient<Arc<dyn JointTrajectoryClient>>>>,
    ik_clients: HashMap<String, ArcIkClient>,
    ik_solvers: HashMap<String, Arc<IkSolverWithChain>>,
    speaker: S,
    move_base: Option<M>,
    navigation: Option<N>,
    joints_poses: HashMap<String, HashMap<String, Vec<f64>>>,
}

impl<S, M, N> RobotClient<S, M, N>
where
    S: Speaker,
    M: MoveBase,
    N: Navigation,
{
    pub fn try_new(
        config: OpenrrClientsConfig,
        raw_joint_trajectory_clients: HashMap<String, Arc<dyn JointTrajectoryClient>>,
        speaker: S,
        move_base: Option<M>,
        navigation: Option<N>,
    ) -> Result<Self, Error> {
        let urdf_full_path = if let Some(p) = config.urdf_full_path().clone() {
            p
        } else {
            return Err(Error::NoUrdfPath);
        };
        debug!("Loading {:?}", urdf_full_path);
        let full_chain_for_collision_checker = Arc::new(Chain::from_urdf_file(&urdf_full_path)?);

        let mut all_joint_trajectory_clients = HashMap::new();
        for (name, client) in &raw_joint_trajectory_clients {
            all_joint_trajectory_clients.insert(name.to_owned(), client.clone());
        }
        for container_config in &config.joint_trajectory_clients_container_configs {
            let mut clients = vec![];
            for name in &container_config.clients_names {
                clients.push(raw_joint_trajectory_clients[name].clone())
            }
            all_joint_trajectory_clients.insert(
                container_config.name.to_owned(),
                Arc::new(JointTrajectoryClientsContainer::new(clients)),
            );
        }

        let collision_check_clients = create_collision_check_clients(
            urdf_full_path,
            &config.self_collision_check_pairs,
            &config.collision_check_clients_configs,
            &all_joint_trajectory_clients,
            full_chain_for_collision_checker.clone(),
        );

        for (name, client) in &collision_check_clients {
            all_joint_trajectory_clients.insert(name.to_owned(), client.clone());
        }

        let ik_clients = create_ik_clients(
            &config.ik_clients_configs,
            &all_joint_trajectory_clients,
            &full_chain_for_collision_checker,
        );

        for (name, client) in &ik_clients {
            all_joint_trajectory_clients.insert(name.to_owned(), client.clone());
        }

        let mut ik_solvers = HashMap::new();
        for (k, i) in &ik_clients {
            ik_solvers.insert(k.to_owned(), i.ik_solver_with_chain.clone());
        }

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
            ik_solvers,
            speaker,
            move_base,
            navigation,
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
    pub fn joint_trajectory_clients(&self) -> &HashMap<String, Arc<dyn JointTrajectoryClient>> {
        &self.all_joint_trajectory_clients
    }
    pub fn ik_solvers(&self) -> &HashMap<String, Arc<IkSolverWithChain>> {
        &self.ik_solvers
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
}

#[async_trait]
impl<S, M, N> Navigation for RobotClient<S, M, N>
where
    S: Speaker,
    M: MoveBase,
    N: Navigation,
{
    async fn send_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: std::time::Duration,
    ) -> Result<(), ArciError> {
        Ok(self
            .navigation
            .as_ref()
            .unwrap()
            .send_pose(goal, frame_id, timeout)
            .await?)
    }
    fn current_pose(&self) -> Result<Isometry2<f64>, ArciError> {
        Ok(self.navigation.as_ref().unwrap().current_pose()?)
    }
    fn cancel(&self) -> Result<(), ArciError> {
        Ok(self.navigation.as_ref().unwrap().cancel()?)
    }
}

impl<S, M, N> MoveBase for RobotClient<S, M, N>
where
    S: Speaker,
    M: MoveBase,
    N: Navigation,
{
    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), ArciError> {
        self.move_base.as_ref().unwrap().send_velocity(velocity)
    }
    fn current_velocity(&self) -> Result<BaseVelocity, ArciError> {
        self.move_base.as_ref().unwrap().current_velocity()
    }
}

impl<S, M, N> Speaker for RobotClient<S, M, N>
where
    S: Speaker,
    M: MoveBase,
    N: Navigation,
{
    fn speak(&self, message: &str) {
        self.speaker.speak(message)
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct JointTrajectoryClientsContainerConfig {
    pub name: String,
    pub clients_names: Vec<String>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct OpenrrClientsConfig {
    pub urdf_path: String,
    urdf_full_path: Option<PathBuf>,
    pub self_collision_check_pairs: Vec<String>,

    pub joint_trajectory_clients_container_configs: Vec<JointTrajectoryClientsContainerConfig>,
    pub collision_check_clients_configs: Vec<CollisionCheckClientConfig>,
    pub ik_clients_configs: Vec<IkClientConfig>,

    pub joints_poses: Vec<JointsPose>,
}

pub fn resolve_relative_path<P: AsRef<Path>>(
    base_path: P,
    path: &str,
) -> Result<Option<PathBuf>, Error> {
    Ok(Some(
        base_path
            .as_ref()
            .parent()
            .ok_or_else(|| Error::NoParentDirectory(base_path.as_ref().to_owned()))?
            .join(path),
    ))
}

impl OpenrrClientsConfig {
    pub fn resolve_path<P: AsRef<Path>>(&mut self, path: P) -> Result<(), Error> {
        self.urdf_full_path = resolve_relative_path(path, &self.urdf_path)?;
        Ok(())
    }
    pub fn urdf_full_path(&self) -> &Option<PathBuf> {
        &self.urdf_full_path
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct JointsPose {
    pub pose_name: String,
    pub client_name: String,
    pub positions: Vec<f64>,
}
