use std::{borrow::Cow, collections::HashMap, mem, sync::Arc, thread::sleep, time::Duration};

use anyhow::format_err;
use arci::{
    nalgebra as na, BaseVelocity, JointPositionLimit, JointPositionLimiter, JointTrajectoryClient,
    JointVelocityLimiter, Localization, MoveBase, Navigation, TrajectoryPoint, WaitFuture,
};
use openrr_sleep::ScopedSleep;
use parking_lot::Mutex;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tokio::sync::oneshot;
use tracing::debug;
use url::Url;

use crate::utils::*;

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UrdfVizWebClientConfig {
    pub name: String,
    pub joint_names: Option<Vec<String>>,
    #[serde(default)]
    pub wrap_with_joint_position_limiter: bool,
    #[serde(default)]
    pub wrap_with_joint_velocity_limiter: bool,
    pub joint_velocity_limits: Option<Vec<f64>>,

    // TOML format has a restriction that if a table itself contains tables,
    // all keys with non-table values must be emitted first.
    // Therefore, these fields must be located at the end of the struct.
    pub joint_position_limits: Option<Vec<JointPositionLimit>>,
}

/// Returns a map of clients for each config.
///
/// The key for the map is [the name of the client](UrdfVizWebClientConfig::name),
/// and in case of conflict, it becomes an error.
///
/// Returns empty map when `configs` are empty.
pub fn create_joint_trajectory_clients(
    configs: Vec<UrdfVizWebClientConfig>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error> {
    create_joint_trajectory_clients_inner(configs, urdf_robot, false)
}

/// Returns a map of clients that will be created lazily for each config.
///
/// See [create_joint_trajectory_clients] for more.
pub fn create_joint_trajectory_clients_lazy(
    configs: Vec<UrdfVizWebClientConfig>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error> {
    create_joint_trajectory_clients_inner(configs, urdf_robot, true)
}

fn create_joint_trajectory_clients_inner(
    configs: Vec<UrdfVizWebClientConfig>,
    urdf_robot: Option<&urdf_rs::Robot>,
    lazy: bool,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error> {
    if configs.is_empty() {
        return Ok(HashMap::default());
    }

    let mut clients = HashMap::new();
    let mut urdf_robot = urdf_robot.map(Cow::Borrowed);

    let create_all_client = move || {
        debug!("create_joint_trajectory_clients_inner: creating UrdfVizWebClient");
        let all_client = UrdfVizWebClient::default();
        all_client.run_send_joint_positions_thread();
        Ok(all_client)
    };
    let all_client: Arc<dyn JointTrajectoryClient> = if lazy && urdf_robot.is_some() {
        let urdf_robot = urdf_robot.as_ref().unwrap();
        Arc::new(arci::Lazy::with_joint_names(
            create_all_client,
            urdf_robot
                .joints
                .iter()
                .filter(|j| j.joint_type != urdf_rs::JointType::Fixed)
                .map(|j| j.name.clone())
                .collect(),
        ))
    } else {
        // Subsequent processing call joint_names, so we cannot make the client
        // lazy if joint_names is not specified.
        let client = create_all_client()?;
        if urdf_robot.is_none()
            && configs.iter().any(|config| {
                config.wrap_with_joint_position_limiter && config.joint_position_limits.is_none()
                    || config.wrap_with_joint_velocity_limiter
                        && config.joint_velocity_limits.is_none()
            })
        {
            urdf_robot = Some(Cow::Owned(client.get_urdf()?));
        }
        Arc::new(client)
    };

    for config in configs {
        if config.wrap_with_joint_position_limiter
            && config.joint_position_limits.is_none()
            && urdf_robot.is_none()
        {
            return Err(format_err!(
                "`wrap_with_joint_position_limiter=true` requires urdf or joint_position_limits \
                is specified",
            )
            .into());
        }
        let client = if let Some(joint_names) = &config.joint_names {
            Arc::new(arci::PartialJointTrajectoryClient::new(
                joint_names.to_owned(),
                all_client.clone(),
            )?)
        } else {
            all_client.clone()
        };
        let client: Arc<dyn JointTrajectoryClient> = if config.wrap_with_joint_velocity_limiter {
            if config.wrap_with_joint_position_limiter {
                Arc::new(new_joint_position_limiter(
                    new_joint_velocity_limiter(
                        client,
                        config.joint_velocity_limits,
                        urdf_robot.as_deref(),
                    )?,
                    config.joint_position_limits,
                    urdf_robot.as_deref(),
                )?)
            } else {
                Arc::new(new_joint_velocity_limiter(
                    client,
                    config.joint_velocity_limits,
                    urdf_robot.as_deref(),
                )?)
            }
        } else if config.wrap_with_joint_position_limiter {
            Arc::new(new_joint_position_limiter(
                client,
                config.joint_position_limits,
                urdf_robot.as_deref(),
            )?)
        } else {
            client
        };
        if clients.insert(config.name.clone(), client).is_some() {
            return Err(
                format_err!("client named '{}' has already been specified", config.name).into(),
            );
        }
    }
    Ok(clients)
}

fn new_joint_position_limiter<C>(
    client: C,
    position_limits: Option<Vec<JointPositionLimit>>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<JointPositionLimiter<C>, arci::Error>
where
    C: JointTrajectoryClient,
{
    match position_limits {
        Some(position_limits) => Ok(JointPositionLimiter::new(client, position_limits)),
        None => JointPositionLimiter::from_urdf(client, &urdf_robot.unwrap().joints),
    }
}

fn new_joint_velocity_limiter<C>(
    client: C,
    velocity_limits: Option<Vec<f64>>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<JointVelocityLimiter<C>, arci::Error>
where
    C: JointTrajectoryClient,
{
    match velocity_limits {
        Some(velocity_limits) => Ok(JointVelocityLimiter::new(client, velocity_limits)),
        None => JointVelocityLimiter::from_urdf(client, &urdf_robot.unwrap().joints),
    }
}

#[derive(Debug, Default)]
enum SendJointPositionsTarget {
    Some {
        trajectory: Vec<TrajectoryPoint>,
        sender: oneshot::Sender<Result<(), arci::Error>>,
    },
    #[default]
    None,
    Abort,
}

impl SendJointPositionsTarget {
    fn set_positions(&mut self, positions: Vec<f64>, duration: Duration) -> WaitFuture {
        self.set_trajectory(vec![TrajectoryPoint::new(positions, duration)])
    }

    fn set_trajectory(&mut self, trajectory: Vec<TrajectoryPoint>) -> WaitFuture {
        let (sender, receiver) = oneshot::channel();
        *self = Self::Some { trajectory, sender };
        WaitFuture::new(async move { receiver.await.map_err(anyhow::Error::from)? })
    }
}

#[derive(Debug, Default)]
struct ThreadState {
    has_send_joint_positions_thread: bool,
    has_send_velocity_thread: bool,
}

#[derive(Clone)]
pub struct UrdfVizWebClient(Arc<UrdfVizWebClientInner>);

struct UrdfVizWebClientInner {
    base_url: Url,
    joint_names: Vec<String>,
    velocity: Mutex<BaseVelocity>,
    send_joint_positions_target: Mutex<SendJointPositionsTarget>,
    threads: Mutex<ThreadState>,
}

impl UrdfVizWebClientInner {
    fn is_dropping(self: &Arc<Self>) -> bool {
        let state = self.threads.lock();
        Arc::strong_count(self)
            <= state.has_send_joint_positions_thread as usize
                + state.has_send_velocity_thread as usize
    }
}

impl UrdfVizWebClient {
    pub fn new(base_url: Url) -> Result<Self, anyhow::Error> {
        let joint_state = get_joint_positions(&base_url)?;
        Ok(Self(Arc::new(UrdfVizWebClientInner {
            base_url,
            joint_names: joint_state.names,
            velocity: Mutex::new(BaseVelocity::default()),
            send_joint_positions_target: Mutex::new(Default::default()),
            threads: Mutex::new(ThreadState::default()),
        })))
    }

    pub fn run_send_velocity_thread(&self) {
        if mem::replace(&mut self.0.threads.lock().has_send_velocity_thread, true) {
            panic!("send_velocity_thread is running");
        }

        let inner = self.0.clone();
        std::thread::spawn(move || {
            struct Bomb(Arc<UrdfVizWebClientInner>);
            impl Drop for Bomb {
                fn drop(&mut self) {
                    self.0.threads.lock().has_send_velocity_thread = false;
                    debug!("terminating send_velocity_thread");
                }
            }

            let bomb = Bomb(inner);
            'outer: while !bomb.0.is_dropping() {
                const DT: f64 = 0.02;

                macro_rules! continue_on_err {
                    ($expr:expr $(,)?) => {
                        match $expr {
                            Ok(x) => x,
                            Err(_e) => {
                                continue 'outer;
                            }
                        }
                    };
                }

                let _guard = ScopedSleep::from_secs(DT);
                let velocity = bomb.0.velocity.lock();
                let mut pose = continue_on_err!(get_robot_origin(&bomb.0.base_url));
                let mut rpy = euler_angles_from_quaternion(&pose.quaternion);
                let yaw = rpy.2;
                pose.position[0] += (velocity.x * yaw.cos() - velocity.y * yaw.sin()) * DT;
                pose.position[1] += (velocity.x * yaw.sin() + velocity.y * yaw.cos()) * DT;
                rpy.2 += velocity.theta * DT;
                pose.quaternion = quaternion_from_euler_angles(rpy.0, rpy.1, rpy.2);
                continue_on_err!(send_robot_origin(&bomb.0.base_url, pose));
            }
        });
    }

    pub fn run_send_joint_positions_thread(&self) {
        if mem::replace(
            &mut self.0.threads.lock().has_send_joint_positions_thread,
            true,
        ) {
            panic!("send_joint_positions_thread is running");
        }

        let inner = self.0.clone();
        std::thread::spawn(move || {
            struct Bomb(Arc<UrdfVizWebClientInner>);
            impl Drop for Bomb {
                fn drop(&mut self) {
                    self.0.threads.lock().has_send_joint_positions_thread = false;
                    debug!("terminating send_joint_positions_thread");
                }
            }

            let bomb = Bomb(inner);
            'outer: while !bomb.0.is_dropping() {
                const UNIT_DURATION: Duration = Duration::from_millis(10);
                let prev = mem::take(&mut *bomb.0.send_joint_positions_target.lock());
                let (trajectory, sender) = match prev {
                    SendJointPositionsTarget::Some { trajectory, sender } => (trajectory, sender),
                    SendJointPositionsTarget::None | SendJointPositionsTarget::Abort => {
                        sleep(UNIT_DURATION);
                        continue;
                    }
                };

                macro_rules! continue_on_err {
                    ($expr:expr $(,)?) => {
                        match $expr {
                            Ok(x) => x,
                            Err(e) => {
                                // Ignore error because WaitFuture may have been dropped.
                                let _ = sender.send(Err(e));
                                continue 'outer;
                            }
                        }
                    };
                }

                let mut last_time = Duration::default();
                for target in trajectory {
                    let duration = target.time_from_start - last_time;
                    last_time = target.time_from_start;
                    if duration.as_nanos() == 0 {
                        let start_time = std::time::Instant::now();
                        let target_state = JointState {
                            names: bomb.0.joint_names.clone(),
                            positions: target.positions.clone(),
                        };
                        continue_on_err!(send_joint_positions(&bomb.0.base_url, target_state));

                        let elapsed = start_time.elapsed();
                        if UNIT_DURATION > elapsed {
                            let sleep_duration = UNIT_DURATION - elapsed;
                            sleep(sleep_duration);
                        }
                        continue;
                    }

                    let current = continue_on_err!(get_joint_positions(&bomb.0.base_url)).positions;
                    let duration_sec = duration.as_secs_f64();
                    let unit_sec = UNIT_DURATION.as_secs_f64();
                    let trajectories = continue_on_err!(openrr_planner::interpolate(
                        &[current, target.positions.to_vec()],
                        duration_sec,
                        unit_sec,
                    )
                    .ok_or_else(|| arci::Error::InterpolationError("".to_owned())));

                    for traj in trajectories {
                        if matches!(
                            *bomb.0.send_joint_positions_target.lock(),
                            SendJointPositionsTarget::Some { .. } | SendJointPositionsTarget::Abort
                        ) {
                            debug!("Abort old target");
                            // Ignore error because WaitFuture may have been dropped.
                            let _ = sender.send(Ok(()));
                            continue 'outer;
                        }
                        let start_time = std::time::Instant::now();
                        let target_state = JointState {
                            names: bomb.0.joint_names.clone(),
                            positions: traj.position,
                        };
                        continue_on_err!(send_joint_positions(&bomb.0.base_url, target_state));
                        let elapsed = start_time.elapsed();
                        if UNIT_DURATION > elapsed {
                            let sleep_duration = UNIT_DURATION - elapsed;
                            sleep(sleep_duration);
                        }
                    }
                }
                // Ignore error because WaitFuture may have been dropped.
                let _ = sender.send(Ok(()));
            }
        });
    }

    pub fn abort(&self) {
        *self.0.send_joint_positions_target.lock() = SendJointPositionsTarget::Abort;
    }

    pub fn get_urdf(&self) -> Result<urdf_rs::Robot, arci::Error> {
        get_urdf(&self.0.base_url)
    }
}

impl Default for UrdfVizWebClient {
    fn default() -> Self {
        Self::new(Url::parse("http://127.0.0.1:7777").unwrap()).unwrap()
    }
}

impl JointTrajectoryClient for UrdfVizWebClient {
    fn joint_names(&self) -> Vec<String> {
        self.0.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        Ok(get_joint_positions(&self.0.base_url)?.positions)
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture, arci::Error> {
        if !self.0.threads.lock().has_send_joint_positions_thread {
            panic!("send_joint_positions called without run_send_joint_positions_thread being called first");
        }

        Ok(self
            .0
            .send_joint_positions_target
            .lock()
            .set_positions(positions, duration))
    }

    fn send_joint_trajectory(
        &self,
        trajectory: Vec<TrajectoryPoint>,
    ) -> Result<WaitFuture, arci::Error> {
        if trajectory.is_empty() {
            return Ok(WaitFuture::ready());
        }

        Ok(self
            .0
            .send_joint_positions_target
            .lock()
            .set_trajectory(trajectory))
    }
}

impl Localization for UrdfVizWebClient {
    fn current_pose(&self, _frame_id: &str) -> Result<na::Isometry2<f64>, arci::Error> {
        let pose = get_robot_origin(&self.0.base_url)?;
        let yaw = euler_angles_from_quaternion(&pose.quaternion).2;
        Ok(na::Isometry2::new(
            na::Vector2::new(pose.position[0], pose.position[1]),
            yaw,
        ))
    }
}

impl Navigation for UrdfVizWebClient {
    fn send_goal_pose(
        &self,
        goal: na::Isometry2<f64>,
        _frame_id: &str,
        _timeout: Duration,
    ) -> Result<WaitFuture, arci::Error> {
        // JUMP!
        send_robot_origin(&self.0.base_url, goal.into())?;

        Ok(WaitFuture::ready())
    }

    fn cancel(&self) -> Result<(), arci::Error> {
        todo!()
    }
}

impl MoveBase for UrdfVizWebClient {
    fn send_velocity(&self, velocity: &arci::BaseVelocity) -> Result<(), arci::Error> {
        if !self.0.threads.lock().has_send_velocity_thread {
            panic!("send_velocity called without run_send_velocity_thread being called first");
        }

        // Check that the connection works.
        // TODO: We use this trick because currently there is no way to tell the
        // main thread when an error has occurred in send_velocity_thread.
        get_robot_origin(&self.0.base_url)?;

        *self.0.velocity.lock() = velocity.to_owned();
        Ok(())
    }

    fn current_velocity(&self) -> Result<arci::BaseVelocity, arci::Error> {
        Ok(self.0.velocity.lock().to_owned())
    }
}
