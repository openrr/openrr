use std::{
    collections::HashMap,
    mem,
    sync::{Arc, Mutex},
    thread::sleep,
    time::Duration,
};

use anyhow::format_err;
use arci::{
    BaseVelocity, CompleteCondition, JointPositionLimit, JointPositionLimiter,
    JointTrajectoryClient, JointVelocityLimiter, Localization, MoveBase, Navigation,
    SetCompleteCondition, TotalJointDiffCondition, TrajectoryPoint, WaitFuture,
};
use nalgebra as na;
use openrr_sleep::ScopedSleep;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::debug;
use url::Url;

use crate::utils::*;

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UrdfVizWebClientConfig {
    pub name: String,
    pub joint_names: Vec<String>,
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

pub fn create_joint_trajectory_clients(
    configs: Vec<UrdfVizWebClientConfig>,
    total_complete_allowable_error: f64,
    complete_timeout_sec: f64,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error> {
    create_joint_trajectory_clients_inner(
        configs,
        total_complete_allowable_error,
        complete_timeout_sec,
        urdf_robot,
        false,
    )
}

pub fn create_joint_trajectory_clients_lazy(
    configs: Vec<UrdfVizWebClientConfig>,
    total_complete_allowable_error: f64,
    complete_timeout_sec: f64,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error> {
    create_joint_trajectory_clients_inner(
        configs,
        total_complete_allowable_error,
        complete_timeout_sec,
        urdf_robot,
        true,
    )
}

fn create_joint_trajectory_clients_inner(
    configs: Vec<UrdfVizWebClientConfig>,
    total_complete_allowable_error: f64,
    complete_timeout_sec: f64,
    urdf_robot: Option<&urdf_rs::Robot>,
    lazy: bool,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error> {
    let mut clients = HashMap::new();

    let create_all_client = move || {
        debug!("create_joint_trajectory_clients_inner: creating UrdfVizWebClient");
        let mut all_client = UrdfVizWebClient::default();
        all_client.set_complete_condition(Box::new(TotalJointDiffCondition::new(
            total_complete_allowable_error,
            complete_timeout_sec,
        )));
        all_client.run_send_joint_positions_thread();
        Ok(all_client)
    };
    let all_client: Arc<dyn JointTrajectoryClient> = if lazy {
        Arc::new(arci::Lazy::new(create_all_client))
    } else {
        Arc::new(create_all_client().unwrap())
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
        let client =
            arci::PartialJointTrajectoryClient::new(config.joint_names, all_client.clone());
        let client: Arc<dyn JointTrajectoryClient> = if config.wrap_with_joint_velocity_limiter {
            if config.wrap_with_joint_position_limiter {
                Arc::new(new_joint_position_limiter(
                    new_joint_velocity_limiter(client, config.joint_velocity_limits, urdf_robot)?,
                    config.joint_position_limits,
                    urdf_robot,
                )?)
            } else {
                Arc::new(new_joint_velocity_limiter(
                    client,
                    config.joint_velocity_limits,
                    urdf_robot,
                )?)
            }
        } else if config.wrap_with_joint_position_limiter {
            Arc::new(new_joint_position_limiter(
                client,
                config.joint_position_limits,
                urdf_robot,
            )?)
        } else {
            Arc::new(client)
        };
        clients.insert(config.name, client);
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

#[derive(Debug)]
enum SendJointPositionsTarget {
    Some(Vec<TrajectoryPoint>),
    None,
    Abort,
}

impl SendJointPositionsTarget {
    fn set_positions(&mut self, positions: Vec<f64>, duration: Duration) {
        *self = Self::Some(vec![TrajectoryPoint::new(positions, duration)]);
    }

    fn set_trajectory(&mut self, trajectory: Vec<TrajectoryPoint>) {
        *self = Self::Some(trajectory);
    }
}

impl Default for SendJointPositionsTarget {
    fn default() -> Self {
        Self::None
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
    complete_condition: Mutex<Arc<dyn CompleteCondition>>,
    threads: Mutex<ThreadState>,
}

impl UrdfVizWebClientInner {
    fn is_dropping(self: &Arc<Self>) -> bool {
        let state = self.threads.lock().unwrap();
        Arc::strong_count(self)
            <= state.has_send_joint_positions_thread as usize
                + state.has_send_velocity_thread as usize
    }
}

impl UrdfVizWebClient {
    pub fn try_new(base_url: Url) -> Result<Self, anyhow::Error> {
        let joint_state = get_joint_positions(&base_url)?;
        Ok(Self(Arc::new(UrdfVizWebClientInner {
            base_url,
            joint_names: joint_state.names,
            velocity: Mutex::new(BaseVelocity::default()),
            send_joint_positions_target: Mutex::new(Default::default()),
            complete_condition: Mutex::new(Arc::new(TotalJointDiffCondition::default())),
            threads: Mutex::new(ThreadState::default()),
        })))
    }

    pub fn run_send_velocity_thread(&self) {
        if mem::replace(
            &mut self.0.threads.lock().unwrap().has_send_velocity_thread,
            true,
        ) {
            panic!("send_velocity_thread is running");
        }

        let inner = self.0.clone();
        std::thread::spawn(move || {
            struct Bomb(Arc<UrdfVizWebClientInner>);
            impl Drop for Bomb {
                fn drop(&mut self) {
                    self.0.threads.lock().unwrap().has_send_velocity_thread = false;
                    debug!("terminating send_velocity_thread");
                }
            }

            let bomb = Bomb(inner);
            while !bomb.0.is_dropping() {
                const DT: f64 = 0.02;
                let _ = ScopedSleep::from_secs(DT);
                let velocity = bomb.0.velocity.lock().unwrap();
                let mut pose = get_robot_origin(&bomb.0.base_url).unwrap();
                let mut rpy = euler_angles_from_quaternion(&pose.quaternion);
                let yaw = rpy.2;
                pose.position[0] += (velocity.x * yaw.cos() - velocity.y * yaw.sin()) * DT;
                pose.position[1] += (velocity.x * yaw.sin() + velocity.y * yaw.cos()) * DT;
                rpy.2 += velocity.theta * DT;
                pose.quaternion = quaternion_from_euler_angles(rpy.0, rpy.1, rpy.2);
                send_robot_origin(&bomb.0.base_url, pose).unwrap();
            }
        });
    }

    pub fn run_send_joint_positions_thread(&self) {
        if mem::replace(
            &mut self
                .0
                .threads
                .lock()
                .unwrap()
                .has_send_joint_positions_thread,
            true,
        ) {
            panic!("send_joint_positions_thread is running");
        }

        let inner = self.0.clone();
        std::thread::spawn(move || {
            struct Bomb(Arc<UrdfVizWebClientInner>);
            impl Drop for Bomb {
                fn drop(&mut self) {
                    self.0
                        .threads
                        .lock()
                        .unwrap()
                        .has_send_joint_positions_thread = false;
                    debug!("terminating send_joint_positions_thread");
                }
            }

            let bomb = Bomb(inner);
            'outer: while !bomb.0.is_dropping() {
                const UNIT_DURATION: Duration = Duration::from_millis(10);
                let trajectory =
                    match { mem::take(&mut *bomb.0.send_joint_positions_target.lock().unwrap()) } {
                        SendJointPositionsTarget::Some(trajectory) => trajectory,
                        SendJointPositionsTarget::None | SendJointPositionsTarget::Abort => {
                            sleep(UNIT_DURATION);
                            continue;
                        }
                    };

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
                        send_joint_positions(&bomb.0.base_url, target_state)
                            .map_err(|e| arci::Error::Connection {
                                message: format!("{:?}", e),
                            })
                            .unwrap();
                        let elapsed = start_time.elapsed();
                        if UNIT_DURATION > elapsed {
                            let sleep_duration = UNIT_DURATION - elapsed;
                            sleep(sleep_duration);
                        }
                        continue;
                    }

                    let current = get_joint_positions(&bomb.0.base_url)
                        .map_err(|e| arci::Error::Connection {
                            message: format!("{:?}", e),
                        })
                        .unwrap()
                        .positions;
                    let duration_sec = duration.as_secs_f64();
                    let unit_sec = UNIT_DURATION.as_secs_f64();
                    let trajectories = openrr_planner::interpolate(
                        &[current, target.positions.to_vec()],
                        duration_sec,
                        unit_sec,
                    )
                    .ok_or_else(|| arci::Error::InterpolationError("".to_owned()))
                    .unwrap();

                    for traj in trajectories {
                        if matches!(
                            *bomb.0.send_joint_positions_target.lock().unwrap(),
                            SendJointPositionsTarget::Some(..) | SendJointPositionsTarget::Abort
                        ) {
                            debug!("Abort old target");
                            break 'outer;
                        }
                        let start_time = std::time::Instant::now();
                        let target_state = JointState {
                            names: bomb.0.joint_names.clone(),
                            positions: traj.position,
                        };
                        send_joint_positions(&bomb.0.base_url, target_state)
                            .map_err(|e| arci::Error::Connection {
                                message: format!("{:?}", e),
                            })
                            .unwrap();
                        let elapsed = start_time.elapsed();
                        if UNIT_DURATION > elapsed {
                            let sleep_duration = UNIT_DURATION - elapsed;
                            sleep(sleep_duration);
                        }
                    }
                }
            }
        });
    }

    pub fn abort(&self) {
        *self.0.send_joint_positions_target.lock().unwrap() = SendJointPositionsTarget::Abort;
    }
}

impl Default for UrdfVizWebClient {
    fn default() -> Self {
        Self::try_new(Url::parse("http://127.0.0.1:7777").unwrap()).unwrap()
    }
}

impl JointTrajectoryClient for UrdfVizWebClient {
    fn joint_names(&self) -> Vec<String> {
        self.0.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        Ok(get_joint_positions(&self.0.base_url)
            .map_err(|e| arci::Error::Connection {
                message: format!("{:?}", e),
            })?
            .positions)
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture, arci::Error> {
        if !self
            .0
            .threads
            .lock()
            .unwrap()
            .has_send_joint_positions_thread
        {
            panic!("send_joint_positions called without run_send_joint_positions_thread being called first");
        }

        self.0
            .send_joint_positions_target
            .lock()
            .unwrap()
            .set_positions(positions.clone(), duration);
        let this = self.clone();
        Ok(WaitFuture::new(async move {
            // Clone to avoid holding the lock for a long time.
            let complete_condition = this.0.complete_condition.lock().unwrap().clone();
            complete_condition
                .wait(&this, &positions, duration.as_secs_f64())
                .await
        }))
    }

    fn send_joint_trajectory(
        &self,
        trajectory: Vec<TrajectoryPoint>,
    ) -> Result<WaitFuture, arci::Error> {
        if trajectory.is_empty() {
            return Ok(WaitFuture::ready());
        }

        let last_traj = trajectory.last().unwrap().clone();
        self.0
            .send_joint_positions_target
            .lock()
            .unwrap()
            .set_trajectory(trajectory);

        let this = self.clone();
        Ok(WaitFuture::new(async move {
            // Clone to avoid holding the lock for a long time.
            let complete_condition = this.0.complete_condition.lock().unwrap().clone();
            complete_condition
                .wait(
                    &this,
                    &last_traj.positions,
                    last_traj.time_from_start.as_secs_f64(),
                )
                .await
        }))
    }
}

impl Localization for UrdfVizWebClient {
    fn current_pose(&self, _frame_id: &str) -> Result<na::Isometry2<f64>, arci::Error> {
        let pose = get_robot_origin(&self.0.base_url).map_err(|e| arci::Error::Connection {
            message: format!("base_url:{}: {:?}", self.0.base_url, e),
        })?;
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
        let re = send_robot_origin(&self.0.base_url, goal.into()).map_err(|e| {
            arci::Error::Connection {
                message: format!("base_url:{}: {:?}", self.0.base_url, e),
            }
        })?;
        if !re.is_ok {
            return Err(arci::Error::Connection { message: re.reason });
        }

        Ok(WaitFuture::ready())
    }

    fn cancel(&self) -> Result<(), arci::Error> {
        todo!()
    }
}

impl MoveBase for UrdfVizWebClient {
    fn send_velocity(&self, velocity: &arci::BaseVelocity) -> Result<(), arci::Error> {
        if !self.0.threads.lock().unwrap().has_send_velocity_thread {
            panic!("send_velocity called without run_send_velocity_thread being called first");
        }

        *self.0.velocity.lock().expect("failed to lock velocity") = velocity.to_owned();
        Ok(())
    }

    fn current_velocity(&self) -> Result<arci::BaseVelocity, arci::Error> {
        Ok(self
            .0
            .velocity
            .lock()
            .expect("failed to lock velocity")
            .to_owned())
    }
}

impl SetCompleteCondition for UrdfVizWebClient {
    fn set_complete_condition(&mut self, condition: Box<dyn CompleteCondition>) {
        *self.0.complete_condition.lock().unwrap() = condition.into();
    }
}
