use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::Duration,
};

use anyhow::format_err;
use arci::{
    copy_joint_positions, CompleteCondition, EachJointDiffCondition, JointPositionLimit,
    JointPositionLimiter, JointTrajectoryClient, JointVelocityLimiter, SetCompleteCondition,
    TotalJointDiffCondition, TrajectoryPoint, WaitFuture,
};
use msg::{
    control_msgs::JointTrajectoryControllerState,
    trajectory_msgs::{JointTrajectory, JointTrajectoryPoint},
};
use once_cell::sync::Lazy;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::{error::Error, msg, SubscriberHandler};

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RosControlClientConfig {
    pub name: String,
    pub joint_names: Vec<String>,
    #[serde(default)]
    pub wrap_with_joint_position_limiter: bool,
    #[serde(default)]
    pub wrap_with_joint_velocity_limiter: bool,
    pub joint_velocity_limits: Option<Vec<f64>>,

    pub controller_name: String,
    pub state_topic_name: Option<String>,
    #[serde(default)]
    pub send_partial_joints_goal: bool,
    pub complete_allowable_errors: Vec<f64>,
    #[serde(default = "default_complete_timeout_sec")]
    pub complete_timeout_sec: f64,

    // TOML format has a restriction that if a table itself contains tables,
    // all keys with non-table values must be emitted first.
    // Therefore, these fields must be located at the end of the struct.
    pub joint_position_limits: Option<Vec<JointPositionLimit>>,
}

const fn default_complete_timeout_sec() -> f64 {
    10.0
}

type StateSubscriber = Lazy<
    SubscriberHandler<JointTrajectoryControllerState>,
    Box<dyn FnOnce() -> SubscriberHandler<JointTrajectoryControllerState> + Send + Sync>,
>;

pub fn create_joint_trajectory_clients(
    configs: Vec<RosControlClientConfig>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error> {
    create_joint_trajectory_clients_inner(configs, urdf_robot, false)
}

pub fn create_joint_trajectory_clients_lazy(
    configs: Vec<RosControlClientConfig>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error> {
    create_joint_trajectory_clients_inner(configs, urdf_robot, true)
}

fn create_joint_trajectory_clients_inner(
    configs: Vec<RosControlClientConfig>,
    urdf_robot: Option<&urdf_rs::Robot>,
    lazy: bool,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error> {
    let mut clients = HashMap::new();
    let mut state_topic_name_to_subscriber: HashMap<String, Arc<StateSubscriber>> = HashMap::new();
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
        let state_topic_name = if let Some(s) = config.state_topic_name {
            s
        } else {
            RosControlClient::state_topic_name(&config.controller_name)
        };
        let joint_state_subscriber_handler = if let Some(joint_state_subscriber_handler) =
            state_topic_name_to_subscriber.get(&state_topic_name)
        {
            joint_state_subscriber_handler.clone()
        } else {
            let joint_state_subscriber_handler =
                RosControlClient::create_joint_state_subscriber_handler(&state_topic_name);
            state_topic_name_to_subscriber
                .insert(state_topic_name, joint_state_subscriber_handler.clone());
            joint_state_subscriber_handler
        };

        let RosControlClientConfig {
            joint_names,
            controller_name,
            send_partial_joints_goal,
            complete_allowable_errors,
            complete_timeout_sec,
            ..
        } = config;
        let joint_names_clone = joint_names.clone();
        let create_client = move || {
            rosrust::ros_debug!("create_joint_trajectory_clients_inner: creating RosControlClient");
            let mut client = RosControlClient::new_with_joint_state_subscriber_handler(
                joint_names_clone,
                &controller_name,
                send_partial_joints_goal,
                joint_state_subscriber_handler,
            );
            client.set_complete_condition(Box::new(EachJointDiffCondition::new(
                complete_allowable_errors,
                complete_timeout_sec,
            )));
            Ok(client)
        };
        let client: Arc<dyn JointTrajectoryClient> = if lazy {
            Arc::new(arci::Lazy::with_joint_names(create_client, joint_names))
        } else {
            Arc::new(create_client().unwrap())
        };

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

pub fn create_joint_trajectory_message_for_send_joint_positions(
    client: &dyn JointTrajectoryClient,
    state: JointTrajectoryControllerState,
    positions: &[f64],
    duration: Duration,
) -> Result<JointTrajectory, arci::Error> {
    let partial_names = client.joint_names();
    if partial_names.len() != positions.len() {
        return Err(arci::Error::LengthMismatch {
            model: partial_names.len(),
            input: positions.len(),
        });
    }
    // TODO: cache index map and use it
    let full_names = state.joint_names;
    let full_dof = full_names.len();

    let mut full_positions = state.actual.positions;
    copy_joint_positions(&partial_names, positions, &full_names, &mut full_positions)?;

    let point_with_full_positions = JointTrajectoryPoint {
        positions: full_positions,
        // add zero velocity to use cubic interpolation in trajectory_controller
        velocities: vec![0.0; full_dof],
        time_from_start: duration.into(),
        ..Default::default()
    };
    Ok(JointTrajectory {
        joint_names: full_names,
        points: vec![point_with_full_positions],
        ..Default::default()
    })
}

pub fn create_joint_trajectory_message_for_send_joint_trajectory(
    client: &dyn JointTrajectoryClient,
    state: JointTrajectoryControllerState,
    trajectory: &[TrajectoryPoint],
) -> Result<JointTrajectory, arci::Error> {
    // TODO: cache index map and use it
    let current_full_positions = state.actual.positions;
    let full_names = state.joint_names;
    let full_dof = current_full_positions.len();

    Ok(JointTrajectory {
        points: trajectory
            .iter()
            .map(|tp| {
                let mut full_positions = current_full_positions.clone();
                copy_joint_positions(
                    &client.joint_names(),
                    &tp.positions,
                    &full_names,
                    &mut full_positions,
                )?;
                Ok(JointTrajectoryPoint {
                    positions: full_positions,
                    velocities: if let Some(partial_velocities) = &tp.velocities {
                        let mut full_velocities = vec![0.0; full_dof];
                        copy_joint_positions(
                            &client.joint_names(),
                            partial_velocities,
                            &full_names,
                            &mut full_velocities,
                        )?;
                        full_velocities
                    } else {
                        vec![]
                    },
                    time_from_start: tp.time_from_start.into(),
                    ..Default::default()
                })
            })
            .collect::<Result<Vec<_>, arci::Error>>()?,
        joint_names: full_names,
        ..Default::default()
    })
}

pub fn extract_current_joint_positions_from_message(
    client: &dyn JointTrajectoryClient,
    state: JointTrajectoryControllerState,
) -> Result<Vec<f64>, arci::Error> {
    // TODO: cache index map and use it
    let mut result = vec![0.0; client.joint_names().len()];
    copy_joint_positions(
        &state.joint_names,
        &state.actual.positions,
        &client.joint_names(),
        &mut result,
    )?;
    Ok(result)
}

#[derive(Clone)]
pub struct RosControlClient(Arc<RosControlClientInner>);

struct RosControlClientInner {
    joint_names: Vec<String>,
    trajectory_publisher: rosrust::Publisher<JointTrajectory>,
    send_partial_joints_goal: bool,
    joint_state_subscriber_handler: Arc<StateSubscriber>,
    complete_condition: Mutex<Arc<dyn CompleteCondition>>,
}

impl RosControlClient {
    pub fn new_with_joint_state_subscriber_handler(
        joint_names: Vec<String>,
        controller_name: &str,
        send_partial_joints_goal: bool,
        joint_state_subscriber_handler: Arc<StateSubscriber>,
    ) -> Self {
        joint_state_subscriber_handler.wait_message(100);

        let state_joint_names = joint_state_subscriber_handler
            .get()
            .unwrap()
            .unwrap()
            .joint_names;
        for joint_name in &joint_names {
            if !state_joint_names.iter().any(|name| **name == *joint_name) {
                panic!(
                    "Invalid configuration : Joint ({}) is not found in state ({:?})",
                    joint_name, state_joint_names
                );
            }
        }

        let trajectory_publisher =
            rosrust::publish(&format!("{}/command", controller_name), 1).unwrap();

        let rate = rosrust::rate(10.0);
        while rosrust::is_ok() && trajectory_publisher.subscriber_count() == 0 {
            rosrust::ros_info!("waiting trajectory subscriber");
            rate.sleep();
        }

        Self(Arc::new(RosControlClientInner {
            joint_names,
            trajectory_publisher,
            send_partial_joints_goal,
            joint_state_subscriber_handler,
            complete_condition: Mutex::new(Arc::new(TotalJointDiffCondition::default())),
        }))
    }

    pub fn new(
        joint_names: Vec<String>,
        controller_name: &str,
        send_partial_joints_goal: bool,
    ) -> Self {
        let joint_state_topic_name = Self::state_topic_name(controller_name);
        Self::new_with_state_topic_name(
            joint_names,
            controller_name,
            &joint_state_topic_name,
            send_partial_joints_goal,
        )
    }

    pub fn state_topic_name(controller_name: &str) -> String {
        format!("{}/state", controller_name)
    }

    fn create_joint_state_subscriber_handler(
        joint_state_topic_name: impl Into<String>,
    ) -> Arc<StateSubscriber> {
        let joint_state_topic_name = joint_state_topic_name.into();
        Arc::new(Lazy::new(Box::new(move || {
            SubscriberHandler::new(&joint_state_topic_name, 1)
        })))
    }

    pub fn new_with_state_topic_name(
        joint_names: Vec<String>,
        controller_name: &str,
        joint_state_topic_name: &str,
        send_partial_joints_goal: bool,
    ) -> Self {
        let joint_state_subscriber_handler =
            Self::create_joint_state_subscriber_handler(joint_state_topic_name);
        Self::new_with_joint_state_subscriber_handler(
            joint_names,
            controller_name,
            send_partial_joints_goal,
            joint_state_subscriber_handler,
        )
    }

    pub fn get_joint_state(&self) -> Result<JointTrajectoryControllerState, arci::Error> {
        self.0
            .joint_state_subscriber_handler
            .get()?
            .ok_or_else(|| arci::Error::Other(Error::NoJointStateAvailable.into()))
    }

    pub fn joint_state_subscriber_handler(&self) -> &Arc<StateSubscriber> {
        &self.0.joint_state_subscriber_handler
    }
}

impl JointTrajectoryClient for RosControlClient {
    fn joint_names(&self) -> Vec<String> {
        self.0.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        extract_current_joint_positions_from_message(self, self.get_joint_state()?)
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture, arci::Error> {
        let traj = if self.0.send_partial_joints_goal {
            JointTrajectory {
                points: vec![JointTrajectoryPoint {
                    positions: positions.clone(),
                    // add zero velocity to use cubic interpolation in trajectory_controller
                    velocities: vec![0.0; self.joint_names().len()],
                    time_from_start: duration.into(),
                    ..Default::default()
                }],
                joint_names: self.joint_names(),
                ..Default::default()
            }
        } else {
            create_joint_trajectory_message_for_send_joint_positions(
                self,
                self.get_joint_state()?,
                &positions,
                duration,
            )?
        };
        self.0.trajectory_publisher.send(traj).unwrap();
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
        let traj = if self.0.send_partial_joints_goal {
            JointTrajectory {
                points: trajectory
                    .iter()
                    .map(|tp| JointTrajectoryPoint {
                        positions: tp.positions.clone(),
                        velocities: if let Some(velocities) = &tp.velocities {
                            velocities.clone()
                        } else {
                            vec![]
                        },
                        time_from_start: tp.time_from_start.into(),
                        ..Default::default()
                    })
                    .collect(),
                joint_names: self.joint_names(),
                ..Default::default()
            }
        } else {
            create_joint_trajectory_message_for_send_joint_trajectory(
                self,
                self.get_joint_state()?,
                &trajectory,
            )?
        };
        self.0.trajectory_publisher.send(traj).unwrap();
        let this = self.clone();
        Ok(WaitFuture::new(async move {
            // Clone to avoid holding the lock for a long time.
            let complete_condition = this.0.complete_condition.lock().unwrap().clone();
            complete_condition
                .wait(
                    &this,
                    &trajectory.last().unwrap().positions,
                    trajectory.last().unwrap().time_from_start.as_secs_f64(),
                )
                .await
        }))
    }
}

impl SetCompleteCondition for RosControlClient {
    fn set_complete_condition(&mut self, condition: Box<dyn CompleteCondition>) {
        *self.0.complete_condition.lock().unwrap() = condition.into();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_joint_position_limiter() {
        use arci::{DummyJointTrajectoryClient, JointPositionLimit, JointPositionLimiter};

        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned(), "b".to_owned()]);
        let limits = vec![
            JointPositionLimit::new(-5.0, 5.0),
            JointPositionLimit::new(-5.0, 5.0),
        ];

        let result = new_joint_position_limiter(client, Some(limits.clone()), None);
        assert!(result.is_ok());
        let _result = result.unwrap();

        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned(), "b".to_owned()]);
        let _correct = JointPositionLimiter::new(client, limits);
    }

    #[test]
    #[should_panic]
    fn test_new_joint_position_limiter_error() {
        use arci::DummyJointTrajectoryClient;

        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned(), "b".to_owned()]);

        let _ = new_joint_position_limiter(client, None, None);
    }
}
