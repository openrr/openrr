use std::{collections::HashMap, sync::Arc};

use crate::msg;
use crate::{error::Error, SubscriberHandler};
use arci::{
    copy_joint_positions, CompleteCondition, EachJointDiffCondition, JointTrajectoryClient,
    JointVelocityLimiter, SetCompleteCondition, TotalJointDiffCondition, TrajectoryPoint,
    WaitFuture,
};
use serde::{Deserialize, Serialize};
use std::time::Duration;

use msg::control_msgs::JointTrajectoryControllerState;
use msg::trajectory_msgs::JointTrajectory;
use msg::trajectory_msgs::JointTrajectoryPoint;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct RosControlClientConfig {
    pub name: String,
    pub joint_names: Vec<String>,
    #[serde(default)]
    pub wrap_with_joint_velocity_limiter: bool,
    #[serde(default)]
    pub joint_velocity_limits: Vec<f64>,

    pub controller_name: String,
    pub state_topic_name: Option<String>,
    #[serde(default)]
    pub send_partial_joints_goal: bool,
    pub complete_allowable_errors: Vec<f64>,
    #[serde(default = "default_complete_timeout_sec")]
    pub complete_timeout_sec: f64,
}

fn default_complete_timeout_sec() -> f64 {
    10.0
}

type StateSubscriber = SubscriberHandler<JointTrajectoryControllerState>;

pub fn create_joint_trajectory_clients(
    configs: Vec<RosControlClientConfig>,
) -> HashMap<String, Arc<dyn JointTrajectoryClient>> {
    let mut clients = HashMap::new();
    let mut state_topic_name_to_subscriber: HashMap<String, Arc<StateSubscriber>> = HashMap::new();
    for config in configs {
        let state_topic_name = if let Some(s) = config.state_topic_name {
            s
        } else {
            RosControlClient::state_topic_name(&config.controller_name)
        };
        #[allow(clippy::map_entry)]
        let mut client = if state_topic_name_to_subscriber.contains_key(&state_topic_name) {
            RosControlClient::new_with_joint_state_subscriber_handler(
                config.joint_names,
                &config.controller_name,
                config.send_partial_joints_goal,
                state_topic_name_to_subscriber
                    .get(&state_topic_name)
                    .unwrap()
                    .clone(),
            )
        } else {
            let client = RosControlClient::new_with_state_topic_name(
                config.joint_names,
                &config.controller_name,
                &state_topic_name,
                config.send_partial_joints_goal,
            );
            state_topic_name_to_subscriber.insert(
                state_topic_name,
                client.joint_state_subscriber_handler().clone(),
            );
            client
        };
        client.set_complete_condition(Box::new(EachJointDiffCondition::new(
            config.complete_allowable_errors,
            config.complete_timeout_sec,
        )));
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
    copy_joint_positions(&partial_names, positions, &full_names, &mut full_positions);

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
                );
                JointTrajectoryPoint {
                    positions: full_positions,
                    velocities: if let Some(partial_velocities) = &tp.velocities {
                        let mut full_velocities = vec![0.0; full_dof];
                        copy_joint_positions(
                            &client.joint_names(),
                            &partial_velocities,
                            &full_names,
                            &mut full_velocities,
                        );
                        full_velocities
                    } else {
                        vec![]
                    },
                    time_from_start: tp.time_from_start.into(),
                    ..Default::default()
                }
            })
            .collect(),
        joint_names: full_names,
        ..Default::default()
    })
}

pub fn extract_current_joint_positions_from_message(
    client: &dyn JointTrajectoryClient,
    state: JointTrajectoryControllerState,
) -> Vec<f64> {
    // TODO: cache index map and use it
    let mut result = vec![0.0; client.joint_names().len()];
    copy_joint_positions(
        &state.joint_names,
        &state.actual.positions,
        &client.joint_names(),
        &mut result,
    );
    result
}

pub struct RosControlClient {
    joint_names: Vec<String>,
    trajectory_publisher: rosrust::Publisher<JointTrajectory>,
    send_partial_joints_goal: bool,
    joint_state_subscriber_handler: Arc<StateSubscriber>,
    complete_condition: Box<dyn CompleteCondition>,
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
            if state_joint_names
                .iter()
                .find(|name| **name == *joint_name)
                .is_none()
            {
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

        Self {
            joint_names,
            trajectory_publisher,
            send_partial_joints_goal,
            joint_state_subscriber_handler,
            complete_condition: Box::new(TotalJointDiffCondition::default()),
        }
    }
    pub fn new(
        joint_names: Vec<String>,
        controller_name: &str,
        send_partial_joints_goal: bool,
    ) -> Self {
        let joint_state_topic_name = Self::state_topic_name(controller_name);
        let joint_state_subscriber_handler =
            Arc::new(SubscriberHandler::new(&joint_state_topic_name, 1));
        Self::new_with_joint_state_subscriber_handler(
            joint_names,
            controller_name,
            send_partial_joints_goal,
            joint_state_subscriber_handler,
        )
    }
    pub fn state_topic_name(controller_name: &str) -> String {
        format!("{}/state", controller_name)
    }
    pub fn new_with_state_topic_name(
        joint_names: Vec<String>,
        controller_name: &str,
        joint_state_topic_name: &str,
        send_partial_joints_goal: bool,
    ) -> Self {
        let joint_state_subscriber_handler =
            Arc::new(SubscriberHandler::new(joint_state_topic_name, 1));
        Self::new_with_joint_state_subscriber_handler(
            joint_names,
            controller_name,
            send_partial_joints_goal,
            joint_state_subscriber_handler,
        )
    }
    pub fn get_joint_state(&self) -> Result<JointTrajectoryControllerState, arci::Error> {
        self.joint_state_subscriber_handler
            .get()?
            .ok_or_else(|| arci::Error::Other(Error::NoJointStateAvailable.into()))
    }
    pub fn joint_state_subscriber_handler(&self) -> &Arc<StateSubscriber> {
        &self.joint_state_subscriber_handler
    }
}

impl JointTrajectoryClient for RosControlClient {
    fn joint_names(&self) -> &[String] {
        &self.joint_names
    }
    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        Ok(extract_current_joint_positions_from_message(
            self,
            self.get_joint_state()?,
        ))
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture, arci::Error> {
        let traj = if self.send_partial_joints_goal {
            JointTrajectory {
                points: vec![JointTrajectoryPoint {
                    positions: positions.clone(),
                    // add zero velocity to use cubic interpolation in trajectory_controller
                    velocities: vec![0.0; self.joint_names().len()],
                    time_from_start: duration.into(),
                    ..Default::default()
                }],
                joint_names: self.joint_names().to_owned(),
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
        self.trajectory_publisher.send(traj).unwrap();
        Ok(WaitFuture::new(async move {
            self.complete_condition
                .wait(self, &positions, duration.as_secs_f64())
                .await
        }))
    }
    fn send_joint_trajectory(
        &self,
        trajectory: Vec<TrajectoryPoint>,
    ) -> Result<WaitFuture, arci::Error> {
        let traj = if self.send_partial_joints_goal {
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
                joint_names: self.joint_names().to_owned(),
                ..Default::default()
            }
        } else {
            create_joint_trajectory_message_for_send_joint_trajectory(
                self,
                self.get_joint_state()?,
                &trajectory,
            )?
        };
        self.trajectory_publisher.send(traj).unwrap();
        Ok(WaitFuture::new(async move {
            self.complete_condition
                .wait(
                    self,
                    &trajectory.last().unwrap().positions,
                    trajectory.last().unwrap().time_from_start.as_secs_f64(),
                )
                .await
        }))
    }
}

impl SetCompleteCondition for RosControlClient {
    fn set_complete_condition(&mut self, condition: Box<dyn CompleteCondition>) {
        self.complete_condition = condition;
    }
}
