use std::{
    collections::HashMap,
    mem,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread::{sleep, JoinHandle},
    time::Duration,
};

use anyhow::format_err;
use arci::{
    BaseVelocity, CompleteCondition, JointPositionLimit, JointPositionLimiter,
    JointTrajectoryClient, JointVelocityLimiter, Localization, MoveBase, Navigation,
    SetCompleteCondition, TotalJointDiffCondition, WaitFuture,
};
use nalgebra as na;
use openrr_sleep::ScopedSleep;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::debug;
use url::Url;

use crate::utils::*;

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
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
    let mut clients = HashMap::new();
    let mut all_client = UrdfVizWebClient::default();
    all_client.set_complete_condition(Box::new(TotalJointDiffCondition::new(
        total_complete_allowable_error,
        complete_timeout_sec,
    )));
    all_client.run_send_joint_positions_thread();
    let all_client = Arc::new(all_client);
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

struct SendJointPositionsTarget {
    positions: Vec<f64>,
    duration: Duration,
}

enum SendJointPositionsTargetState {
    Some(SendJointPositionsTarget),
    None,
    Abort,
}

impl SendJointPositionsTargetState {
    fn take(&mut self) -> Self {
        mem::replace(self, Self::None)
    }
}

pub struct UrdfVizWebClient {
    base_url: Url,
    joint_names: Vec<String>,
    velocity: Arc<Mutex<BaseVelocity>>,
    send_joint_positions_target: Arc<Mutex<SendJointPositionsTargetState>>,
    complete_condition: Box<dyn CompleteCondition>,
    send_joint_positions_thread: Option<JoinHandle<()>>,
    is_dropping: Arc<AtomicBool>,
}

impl UrdfVizWebClient {
    pub fn try_new(base_url: Url) -> Result<Self, anyhow::Error> {
        let joint_state = get_joint_positions(&base_url)?;
        let velocity = Arc::new(Mutex::new(BaseVelocity::default()));
        let send_joint_positions_target = Arc::new(Mutex::new(SendJointPositionsTargetState::None));
        Ok(Self {
            base_url,
            joint_names: joint_state.names,
            velocity,
            send_joint_positions_target,
            complete_condition: Box::new(TotalJointDiffCondition::default()),
            send_joint_positions_thread: None,
            is_dropping: Arc::new(AtomicBool::new(false)),
        })
    }

    pub fn run_thread(&self) {
        let velocity_arc_mutex = self.velocity.clone();
        let base_url = self.base_url.clone();
        std::thread::spawn(move || loop {
            const DT: f64 = 0.02;
            let _ = ScopedSleep::from_secs(DT);
            let velocity = velocity_arc_mutex.lock().unwrap();
            let mut pose = get_robot_origin(&base_url).unwrap();
            let mut rpy = euler_angles_from_quaternion(&pose.quaternion);
            let yaw = rpy.2;
            pose.position[0] += (velocity.x * yaw.cos() - velocity.y * yaw.sin()) * DT;
            pose.position[1] += (velocity.x * yaw.sin() + velocity.y * yaw.cos()) * DT;
            rpy.2 += velocity.theta * DT;
            pose.quaternion = quaternion_from_euler_angles(rpy.0, rpy.1, rpy.2);
            send_robot_origin(&base_url, pose).unwrap();
        });
    }

    pub fn run_send_joint_positions_thread(&mut self) {
        if self.send_joint_positions_thread.is_some() {
            panic!("send_joint_positions_thread is running.");
        }
        let send_joint_positions_target_arc_mutex = self.send_joint_positions_target.clone();
        let base_url = self.base_url.clone();
        let joint_names = self.joint_names.clone();
        let is_dropping_arc_mutex = self.is_dropping.clone();

        self.send_joint_positions_thread = Some(std::thread::spawn(move || {
            while !is_dropping_arc_mutex.load(Ordering::Relaxed) {
                const UNIT_DURATION: Duration = Duration::from_millis(10);
                let send_joint_positions_target =
                    { send_joint_positions_target_arc_mutex.lock().unwrap().take() };
                if let SendJointPositionsTargetState::Some(target) = send_joint_positions_target {
                    if target.duration.as_nanos() == 0 {
                        let target_state = JointState {
                            names: joint_names.clone(),
                            positions: target.positions.clone(),
                        };
                        send_joint_positions(&base_url, target_state)
                            .map_err(|e| arci::Error::Connection {
                                message: format!("{:?}", e),
                            })
                            .unwrap();
                        continue;
                    }

                    let current = get_joint_positions(&base_url)
                        .map_err(|e| arci::Error::Connection {
                            message: format!("{:?}", e),
                        })
                        .unwrap()
                        .positions;
                    let duration_sec = target.duration.as_secs_f64();
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
                            *send_joint_positions_target_arc_mutex.lock().unwrap(),
                            SendJointPositionsTargetState::Some(..)
                                | SendJointPositionsTargetState::Abort
                        ) {
                            debug!("Abort old target.");
                            break;
                        }
                        let start_time = std::time::Instant::now();
                        let target_state = JointState {
                            names: joint_names.clone(),
                            positions: traj.position,
                        };
                        send_joint_positions(&base_url, target_state)
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
                } else {
                    sleep(UNIT_DURATION);
                }
            }
        }));
    }

    pub fn abort(&self) {
        *self.send_joint_positions_target.lock().unwrap() = SendJointPositionsTargetState::Abort;
    }
}

impl Drop for UrdfVizWebClient {
    fn drop(&mut self) {
        if let Some(t) = self.send_joint_positions_thread.take() {
            self.is_dropping.swap(true, Ordering::Relaxed);
            t.join().unwrap();
        }
    }
}

impl Default for UrdfVizWebClient {
    fn default() -> Self {
        Self::try_new(Url::parse("http://127.0.0.1:7777").unwrap()).unwrap()
    }
}

impl JointTrajectoryClient for UrdfVizWebClient {
    fn joint_names(&self) -> &[String] {
        &self.joint_names
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        Ok(get_joint_positions(&self.base_url)
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
        if self.send_joint_positions_thread.is_none() {
            panic!("Call run_joint_positions_thread.");
        }
        *self.send_joint_positions_target.lock().unwrap() =
            SendJointPositionsTargetState::Some(SendJointPositionsTarget {
                positions: positions.clone(),
                duration,
            });
        Ok(WaitFuture::new(async move {
            self.complete_condition
                .wait(self, &positions, duration.as_secs_f64())
                .await
        }))
    }

    fn send_joint_trajectory(
        &self,
        trajectory: Vec<arci::TrajectoryPoint>,
    ) -> Result<WaitFuture, arci::Error> {
        if trajectory.is_empty() {
            return Ok(WaitFuture::ready());
        }

        let mut last_time = Duration::default();
        let last_traj = trajectory.last().unwrap().clone();
        for traj in trajectory {
            let _ = self.send_joint_positions(traj.positions, traj.time_from_start - last_time)?;
            last_time = traj.time_from_start;
        }
        Ok(WaitFuture::new(async move {
            self.complete_condition
                .wait(
                    self,
                    &last_traj.positions,
                    last_traj.time_from_start.as_secs_f64(),
                )
                .await
        }))
    }
}

impl Localization for UrdfVizWebClient {
    fn current_pose(&self, _frame_id: &str) -> Result<na::Isometry2<f64>, arci::Error> {
        let pose = get_robot_origin(&self.base_url).map_err(|e| arci::Error::Connection {
            message: format!("base_url:{}: {:?}", self.base_url, e),
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
        let re = send_robot_origin(&self.base_url, goal.into()).map_err(|e| {
            arci::Error::Connection {
                message: format!("base_url:{}: {:?}", self.base_url, e),
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
        *self.velocity.lock().expect("failed to lock velocity") = velocity.to_owned();
        Ok(())
    }

    fn current_velocity(&self) -> Result<arci::BaseVelocity, arci::Error> {
        Ok(self
            .velocity
            .lock()
            .expect("failed to lock velocity")
            .to_owned())
    }
}

impl SetCompleteCondition for UrdfVizWebClient {
    fn set_complete_condition(&mut self, condition: Box<dyn CompleteCondition>) {
        self.complete_condition = condition;
    }
}
