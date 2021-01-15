use crate::utils::*;
use arci::{
    BaseVelocity, CompleteCondition, JointTrajectoryClient, JointVelocityLimiter, MoveBase,
    Navigation, SetCompleteCondition, TotalJointDiffCondition,
};
use async_trait::async_trait;
use nalgebra as na;
use openrr_sleep::ScopedSleep;
use serde::{Deserialize, Serialize};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};
use url::Url;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct UrdfVizWebClientConfig {
    pub name: String,
    pub joint_names: Vec<String>,
    pub wrap_with_joint_velocity_limiter: bool,
    pub joint_velocity_limits: Vec<f64>,
}

pub fn create_joint_trajectory_clients(
    configs: Vec<UrdfVizWebClientConfig>,
) -> HashMap<String, Arc<dyn JointTrajectoryClient>> {
    let mut clients = HashMap::new();
    let all_client = Arc::new(UrdfVizWebClient::default());
    for config in configs {
        let client =
            arci::PartialJointTrajectoryClient::new(config.joint_names, all_client.clone());
        let client: Arc<dyn JointTrajectoryClient> = if config.wrap_with_joint_velocity_limiter {
            Arc::new(JointVelocityLimiter::new(
                client,
                config.joint_velocity_limits,
            ))
        } else {
            Arc::new(client)
        };
        clients.insert(config.name, client);
    }
    clients
}

pub struct UrdfVizWebClient {
    base_url: Url,
    joint_names: Vec<String>,
    velocity: Arc<Mutex<BaseVelocity>>,
    complete_condition: Box<dyn CompleteCondition>,
}

impl UrdfVizWebClient {
    pub fn try_new(base_url: Url) -> Result<Self, anyhow::Error> {
        let joint_state = get_joint_positions(&base_url)?;
        let velocity = Arc::new(Mutex::new(BaseVelocity::default()));
        Ok(Self {
            base_url,
            joint_names: joint_state.names,
            velocity,
            complete_condition: Box::new(TotalJointDiffCondition::default()),
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
}

impl Default for UrdfVizWebClient {
    fn default() -> Self {
        Self::try_new(Url::parse("http://127.0.0.1:7777").unwrap()).unwrap()
    }
}

#[async_trait]
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
    async fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<(), arci::Error> {
        const UNIT_DURATION: std::time::Duration = std::time::Duration::from_millis(10);
        let current = self.current_joint_positions()?;
        let duration_sec = duration.as_secs_f64();
        let unit_sec = UNIT_DURATION.as_secs_f64();
        let trajectories =
            openrr_planner::interpolate(&[current, positions.to_vec()], duration_sec, unit_sec)
                .ok_or_else(|| arci::Error::InterpolationError("".to_owned()))?;

        let url = self.base_url.clone();
        let joint_names = self.joint_names.clone();
        let _handle = std::thread::spawn(move || {
            for traj in trajectories {
                let start_time = std::time::Instant::now();
                let target_state = JointState {
                    names: joint_names.clone(),
                    positions: traj.position,
                };
                let re = send_joint_positions(&url, target_state).map_err(|e| {
                    arci::Error::Connection {
                        message: format!("{:?}", e),
                    }
                })?;
                if !re.is_ok {
                    return Err(arci::Error::Connection { message: re.reason });
                }
                let elapsed = start_time.elapsed();
                if UNIT_DURATION > elapsed {
                    let sleep_duration = UNIT_DURATION - elapsed;
                    std::thread::sleep(sleep_duration);
                }
            }
            Ok(())
        });
        self.complete_condition.wait(self, &positions)
    }

    async fn send_joint_trajectory(
        &self,
        trajectory: Vec<arci::TrajectoryPoint>,
    ) -> Result<(), arci::Error> {
        let mut last_time = std::time::Duration::default();
        for traj in trajectory {
            self.send_joint_positions(traj.positions, traj.time_from_start - last_time)
                .await?;
            last_time = traj.time_from_start;
        }
        Ok(())
    }
}

#[async_trait]
impl Navigation for UrdfVizWebClient {
    async fn send_pose(
        &self,
        goal: na::Isometry2<f64>,
        _timeout: std::time::Duration,
    ) -> Result<(), arci::Error> {
        // JUMP!
        let re = send_robot_origin(&self.base_url, goal.into()).map_err(|e| {
            arci::Error::Connection {
                message: format!("base_url:{}: {:?}", self.base_url, e),
            }
        })?;
        if !re.is_ok {
            return Err(arci::Error::Connection { message: re.reason });
        }

        Ok(())
    }

    fn current_pose(&self) -> Result<na::Isometry2<f64>, arci::Error> {
        let pose = get_robot_origin(&self.base_url).map_err(|e| arci::Error::Connection {
            message: format!("base_url:{}: {:?}", self.base_url, e),
        })?;
        let yaw = euler_angles_from_quaternion(&pose.quaternion).2;
        Ok(na::Isometry2::new(
            na::Vector2::new(pose.position[0], pose.position[1]),
            yaw,
        ))
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
