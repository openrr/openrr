use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
    time::Duration,
};

use arci::{
    BaseVelocity, Error as ArciError, JointTrajectoryClient, JointTrajectoryClientsContainer,
    Localization, MoveBase, Navigation, Speaker, WaitFuture,
};
use k::{nalgebra::Isometry2, Chain, Isometry3};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::debug;

use crate::{
    create_collision_check_client, create_ik_solver_with_chain, CollisionCheckClient, Error,
    IkClient, IkSolverConfig, IkSolverWithChain, SelfCollisionChecker, SelfCollisionCheckerConfig,
};

type ArcIkClient = Arc<IkClient<Arc<dyn JointTrajectoryClient>>>;
pub type ArcRobotClient =
    RobotClient<Arc<dyn Localization>, Arc<dyn MoveBase>, Arc<dyn Navigation>>;
pub type BoxRobotClient =
    RobotClient<Box<dyn Localization>, Box<dyn MoveBase>, Box<dyn Navigation>>;

type ArcJointTrajectoryClient = Arc<dyn JointTrajectoryClient>;

pub struct RobotClient<L, M, N>
where
    L: Localization,
    M: MoveBase,
    N: Navigation,
{
    full_chain_for_collision_checker: Option<Arc<Chain<f64>>>,
    raw_joint_trajectory_clients: HashMap<String, Arc<dyn JointTrajectoryClient>>,
    all_joint_trajectory_clients: HashMap<String, Arc<dyn JointTrajectoryClient>>,
    collision_check_clients:
        HashMap<String, Arc<CollisionCheckClient<Arc<dyn JointTrajectoryClient>>>>,
    ik_clients: HashMap<String, ArcIkClient>,
    self_collision_checkers: HashMap<String, Arc<SelfCollisionChecker>>,
    ik_solvers: HashMap<String, Arc<IkSolverWithChain>>,
    speakers: HashMap<String, Arc<dyn Speaker>>,
    localization: Option<L>,
    move_base: Option<M>,
    navigation: Option<N>,
    joints_poses: HashMap<String, HashMap<String, Vec<f64>>>,
}

impl<L, M, N> RobotClient<L, M, N>
where
    L: Localization,
    M: MoveBase,
    N: Navigation,
{
    pub fn new(
        config: OpenrrClientsConfig,
        raw_joint_trajectory_clients: HashMap<String, Arc<dyn JointTrajectoryClient>>,
        speakers: HashMap<String, Arc<dyn Speaker>>,
        localization: Option<L>,
        move_base: Option<M>,
        navigation: Option<N>,
    ) -> Result<Self, Error> {
        debug!("{:?}", config);

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

        let (
            full_chain_for_collision_checker,
            collision_check_clients,
            ik_clients,
            self_collision_checkers,
            ik_solvers,
        ) = if let Some(urdf_full_path) = config.urdf_full_path() {
            debug!("Loading {:?}", urdf_full_path);
            let full_chain_for_collision_checker =
                Arc::new(Chain::from_urdf_file(&urdf_full_path)?);

            let collision_check_clients = create_collision_check_clients(
                urdf_full_path,
                &config.self_collision_check_pairs,
                &config.collision_check_clients_configs,
                &all_joint_trajectory_clients,
                full_chain_for_collision_checker.clone(),
            );

            let mut self_collision_checkers = HashMap::new();

            for (name, client) in &collision_check_clients {
                self_collision_checkers.insert(name.to_owned(), client.collision_checker.clone());
                all_joint_trajectory_clients.insert(name.to_owned(), client.clone());
            }

            let mut ik_solvers = HashMap::new();
            for (k, c) in &config.ik_solvers_configs {
                ik_solvers.insert(
                    k.to_owned(),
                    Arc::new(create_ik_solver_with_chain(
                        &full_chain_for_collision_checker,
                        c,
                    )),
                );
            }

            let ik_clients = create_ik_clients(
                &config.ik_clients_configs,
                &all_joint_trajectory_clients,
                &ik_solvers,
            );

            for (name, client) in &ik_clients {
                all_joint_trajectory_clients.insert(name.to_owned(), client.clone());
            }
            (
                Some(full_chain_for_collision_checker),
                collision_check_clients,
                ik_clients,
                self_collision_checkers,
                ik_solvers,
            )
        } else {
            (
                None,
                HashMap::new(),
                HashMap::new(),
                HashMap::new(),
                HashMap::new(),
            )
        };
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
            self_collision_checkers,
            ik_solvers,
            speakers,
            localization,
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
                if let Some(joint) = self
                    .full_chain_for_collision_checker
                    .as_ref()
                    .unwrap()
                    .find(joint_name)
                {
                    joint.set_joint_position_clamped(positions[index])
                } else {
                    return Err(ArciError::NoJoint(joint_name.to_owned()).into());
                }
            }
        }
        self.full_chain_for_collision_checker
            .as_ref()
            .unwrap()
            .update_transforms();
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

    pub fn joint_trajectory_clients(&self) -> &HashMap<String, Arc<dyn JointTrajectoryClient>> {
        &self.all_joint_trajectory_clients
    }

    pub fn self_collision_checkers(&self) -> &HashMap<String, Arc<SelfCollisionChecker>> {
        &self.self_collision_checkers
    }

    pub fn ik_solvers(&self) -> &HashMap<String, Arc<IkSolverWithChain>> {
        &self.ik_solvers
    }

    pub fn ik_clients(&self) -> &HashMap<String, ArcIkClient> {
        &self.ik_clients
    }

    pub fn send_joint_positions(
        &self,
        name: &str,
        positions: &[f64],
        duration_sec: f64,
    ) -> Result<WaitFuture, Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        if self.is_ik_client(name) {
            Ok(self.ik_client(name)?.client.send_joint_positions(
                positions.to_owned(),
                Duration::from_secs_f64(duration_sec),
            )?)
        } else {
            Ok(self.joint_trajectory_client(name)?.send_joint_positions(
                positions.to_owned(),
                Duration::from_secs_f64(duration_sec),
            )?)
        }
    }

    pub fn current_joint_positions(&self, name: &str) -> Result<Vec<f64>, Error> {
        if self.is_ik_client(name) {
            self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
            Ok(self.ik_client(name)?.current_joint_positions()?)
        } else {
            Ok(self
                .joint_trajectory_client(name)?
                .current_joint_positions()?)
        }
    }

    pub fn send_joints_pose(
        &self,
        name: &str,
        pose_name: &str,
        duration_sec: f64,
    ) -> Result<WaitFuture, Error> {
        if self.joints_poses.contains_key(name) && self.joints_poses[name].contains_key(pose_name) {
            Ok(self.send_joint_positions(
                name,
                &self.joints_poses[name][pose_name],
                duration_sec,
            )?)
        } else {
            Err(Error::NoJointsPose(name.to_owned(), pose_name.to_owned()))
        }
    }

    pub fn current_end_transform(&self, name: &str) -> Result<Isometry3<f64>, Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        Ok(self.ik_client(name)?.current_end_transform()?)
    }

    pub fn transform(&self, name: &str, pose: &Isometry3<f64>) -> Result<Isometry3<f64>, Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        Ok(self.ik_client(name)?.transform(pose)?)
    }

    pub fn move_ik(
        &self,
        name: &str,
        target_pose: &Isometry3<f64>,
        duration_sec: f64,
    ) -> Result<WaitFuture, Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        Ok(self.ik_client(name)?.move_ik(target_pose, duration_sec)?)
    }

    pub fn move_ik_with_interpolation(
        &self,
        name: &str,
        target_pose: &Isometry3<f64>,
        duration_sec: f64,
    ) -> Result<WaitFuture, Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        Ok(self
            .ik_client(name)?
            .move_ik_with_interpolation(target_pose, duration_sec)?)
    }

    pub fn send_joint_positions_with_pose_interpolation(
        &self,
        name: &str,
        positions: &[f64],
        duration_sec: f64,
    ) -> Result<WaitFuture, Error> {
        self.set_raw_clients_joint_positions_to_full_chain_for_collision_checker()?;
        let target_pose = {
            let ik_client = self.ik_client(name)?;
            ik_client.set_joint_positions_clamped(positions);
            ik_client.ik_solver_with_chain.end_transform()
        };
        self.move_ik_with_interpolation(name, &target_pose, duration_sec)
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

    pub fn full_chain_for_collision_checker(&self) -> &Option<Arc<Chain<f64>>> {
        &self.full_chain_for_collision_checker
    }

    pub fn speakers(&self) -> &HashMap<String, Arc<dyn Speaker>> {
        &self.speakers
    }

    pub fn speak(&self, name: &str, message: &str) -> Result<WaitFuture, Error> {
        match self.speakers.get(&name.to_string()) {
            Some(speaker) => Ok(speaker.speak(message)?),
            _ => Err(anyhow::format_err!("Speaker \"{}\" is not found.", name).into()),
        }
    }
}

impl<L, M, N> Localization for RobotClient<L, M, N>
where
    L: Localization,
    M: MoveBase,
    N: Navigation,
{
    fn current_pose(&self, frame_id: &str) -> Result<Isometry2<f64>, ArciError> {
        self.localization.as_ref().unwrap().current_pose(frame_id)
    }
}

impl<L, M, N> Navigation for RobotClient<L, M, N>
where
    L: Localization,
    M: MoveBase,
    N: Navigation,
{
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: std::time::Duration,
    ) -> Result<WaitFuture, ArciError> {
        self.navigation
            .as_ref()
            .unwrap()
            .send_goal_pose(goal, frame_id, timeout)
    }

    fn cancel(&self) -> Result<(), ArciError> {
        self.navigation.as_ref().unwrap().cancel()
    }
}

impl<L, M, N> MoveBase for RobotClient<L, M, N>
where
    L: Localization,
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

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct JointTrajectoryClientsContainerConfig {
    pub name: String,
    pub clients_names: Vec<String>,
}

#[derive(Debug, Serialize, Deserialize, Clone, Default, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct OpenrrClientsConfig {
    pub urdf_path: Option<String>,
    urdf_full_path: Option<PathBuf>,
    #[serde(default)]
    pub self_collision_check_pairs: Vec<String>,

    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub joint_trajectory_clients_container_configs: Vec<JointTrajectoryClientsContainerConfig>,
    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub collision_check_clients_configs: Vec<CollisionCheckClientConfig>,
    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub ik_clients_configs: Vec<IkClientConfig>,
    #[serde(default)]
    pub ik_solvers_configs: HashMap<String, IkSolverConfig>,

    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub joints_poses: Vec<JointsPose>,
}

/// Make relative path into absolute path from base file (not base dir).
///
/// # Example
/// ```
/// use std::path::PathBuf;
/// let abs_path = openrr_client::resolve_relative_path("/home/a/base_file.toml", "../another_file.mp3").unwrap();
/// assert_eq!(abs_path, PathBuf::from("/home/a/../another_file.mp3"));
/// ```
pub fn resolve_relative_path<B: AsRef<Path>, P: AsRef<Path>>(
    base_path: B,
    path: P,
) -> Result<PathBuf, Error> {
    Ok(base_path
        .as_ref()
        .parent()
        .ok_or_else(|| Error::NoParentDirectory(base_path.as_ref().to_owned()))?
        .join(path))
}

impl OpenrrClientsConfig {
    pub fn resolve_path<P: AsRef<Path>>(&mut self, path: P) -> Result<(), Error> {
        if let Some(urdf_path) = self.urdf_path.as_ref() {
            self.urdf_full_path = Some(resolve_relative_path(path, &urdf_path)?);
        } else {
            return Err(Error::NoUrdfPath);
        }
        Ok(())
    }

    pub fn urdf_full_path(&self) -> Option<&Path> {
        self.urdf_full_path.as_deref()
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct JointsPose {
    pub pose_name: String,
    pub client_name: String,
    pub positions: Vec<f64>,
}

#[derive(Clone, Serialize, Deserialize, Debug, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct IkClientConfig {
    pub name: String,
    pub client_name: String,
    pub solver_name: String,
}

pub fn create_ik_clients(
    configs: &[IkClientConfig],
    name_to_joint_trajectory_client: &HashMap<String, ArcJointTrajectoryClient>,
    name_to_ik_solvers: &HashMap<String, Arc<IkSolverWithChain>>,
) -> HashMap<String, Arc<IkClient<ArcJointTrajectoryClient>>> {
    let mut clients = HashMap::new();
    for config in configs {
        clients.insert(
            config.name.clone(),
            Arc::new(IkClient::new(
                name_to_joint_trajectory_client[&config.client_name].clone(),
                name_to_ik_solvers[&config.solver_name].clone(),
            )),
        );
    }
    clients
}

#[derive(Clone, Serialize, Deserialize, Debug, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CollisionCheckClientConfig {
    pub name: String,
    pub client_name: String,
    #[serde(default)]
    pub self_collision_checker_config: SelfCollisionCheckerConfig,
}

pub fn create_collision_check_clients<P: AsRef<Path>>(
    urdf_path: P,
    self_collision_check_pairs: &[String],
    configs: &[CollisionCheckClientConfig],
    name_to_joint_trajectory_client: &HashMap<String, Arc<dyn JointTrajectoryClient>>,
    full_chain: Arc<k::Chain<f64>>,
) -> HashMap<String, Arc<CollisionCheckClient<Arc<dyn JointTrajectoryClient>>>> {
    let mut clients = HashMap::new();
    for config in configs {
        clients.insert(
            config.name.clone(),
            Arc::new(create_collision_check_client(
                &urdf_path,
                self_collision_check_pairs,
                &config.self_collision_checker_config,
                name_to_joint_trajectory_client[&config.client_name].clone(),
                full_chain.clone(),
            )),
        );
    }
    clients
}

#[cfg(test)]
mod tests {
    use super::*;

    struct PanicJointTrajectoryClient;

    impl JointTrajectoryClient for PanicJointTrajectoryClient {
        #[track_caller]
        fn joint_names(&self) -> Vec<String> {
            // TODO
            // panic!("PanicJointTrajectoryClient::joint_names")
            vec![
                "l_shoulder_yaw".into(),
                "l_shoulder_pitch".into(),
                "l_shoulder_roll".into(),
                "l_elbow_pitch".into(),
                "l_wrist_yaw".into(),
                "l_wrist_pitch".into(),
            ]
        }

        #[track_caller]
        fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
            panic!("PanicJointTrajectoryClient::current_joint_positions")
        }

        #[track_caller]
        fn send_joint_positions(
            &self,
            positions: Vec<f64>,
            duration: std::time::Duration,
        ) -> Result<WaitFuture, arci::Error> {
            panic!(
                "PanicJointTrajectoryClient::send_joint_positions positions={:?}, duration={:?}",
                positions, duration
            )
        }

        #[track_caller]
        fn send_joint_trajectory(
            &self,
            trajectory: Vec<arci::TrajectoryPoint>,
        ) -> Result<WaitFuture, arci::Error> {
            panic!(
                "PanicJointTrajectoryClient::send_joint_trajectory trajectory={:?}",
                trajectory
            )
        }
    }

    struct PanicSpeaker;

    impl Speaker for PanicSpeaker {
        #[track_caller]
        fn speak(&self, message: &str) -> Result<WaitFuture, arci::Error> {
            panic!("PanicSpeaker::speak message={:?}", message)
        }
    }

    struct PanicLocalization;

    impl Localization for PanicLocalization {
        #[track_caller]
        fn current_pose(&self, frame_id: &str) -> Result<arci::Isometry2<f64>, arci::Error> {
            panic!("PanicLocalization::current_pose frame_id={:?}", frame_id)
        }
    }

    struct PanicMoveBase;

    impl MoveBase for PanicMoveBase {
        #[track_caller]
        fn current_velocity(&self) -> Result<BaseVelocity, arci::Error> {
            panic!("PanicMoveBase::current_velocity")
        }

        #[track_caller]
        fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), arci::Error> {
            panic!("PanicMoveBase::send_velocity velocity={:?}", velocity)
        }
    }

    struct PanicNavigation;

    impl Navigation for PanicNavigation {
        #[track_caller]
        fn send_goal_pose(
            &self,
            goal: Isometry2<f64>,
            frame_id: &str,
            timeout: std::time::Duration,
        ) -> Result<WaitFuture, arci::Error> {
            panic!(
                "PanicNavigation::current_pose goal={:?}, frame_id={:?}, timeout={:?}",
                goal, frame_id, timeout
            )
        }

        #[track_caller]
        fn cancel(&self) -> Result<(), arci::Error> {
            panic!("PanicNavigation::cancel")
        }
    }

    #[test]
    fn lazy() {
        let mut root_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        root_dir.pop(); // openrr-config

        let mut config: OpenrrClientsConfig = toml::from_str(&format!(
            r#"
urdf_path = "{}/openrr-planner/sample.urdf"
self_collision_check_pairs = ["l_shoulder_yaw:l_gripper_linear1"]

[[joint_trajectory_clients_container_configs]]
name = "arm"
clients_names = ["arm"]

[[collision_check_clients_configs]]
name = "arm_collision_checked"
client_name = "arm"

[[ik_clients_configs]]
name = "arm_ik"
client_name = "arm_collision_checked"
solver_name = "arm_ik_solver"

[ik_solvers_configs.arm_ik_solver]
ik_target = "l_tool_fixed"

[[joints_poses]]
pose_name = "zero"
client_name = "arm"
positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
"#,
            root_dir.display()
        ))
        .unwrap();
        config.urdf_full_path = Some(root_dir.join("openrr-planner/sample.urdf"));
        let _client = BoxRobotClient::new(
            config,
            {
                let mut map = HashMap::new();
                map.insert(
                    "arm".to_string(),
                    Arc::new(PanicJointTrajectoryClient) as Arc<dyn JointTrajectoryClient>,
                );
                map.insert(
                    "torso".to_string(),
                    Arc::new(PanicJointTrajectoryClient) as Arc<dyn JointTrajectoryClient>,
                );
                map
            },
            {
                let mut map = HashMap::new();
                map.insert("a".to_string(), Arc::new(PanicSpeaker) as Arc<dyn Speaker>);
                map
            },
            Some(Box::new(PanicLocalization)),
            Some(Box::new(PanicMoveBase)),
            Some(Box::new(PanicNavigation)),
        )
        .unwrap();
    }
}
