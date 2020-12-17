use crate::error::Error;
use crate::msg;
use crate::rosrust_utils::*;
use arci::{
    copy_joint_positions, CompleteCondition, JointTrajectoryClient, SetCompleteCondition,
    TotalJointDiffCondition, TrajectoryPoint,
};
use async_trait::async_trait;

use msg::control_msgs::JointTrajectoryControllerState;
use msg::trajectory_msgs::JointTrajectory as RosJointTrajectory;

pub fn create_joint_trajectory_message_for_send_joint_positions(
    client: &dyn JointTrajectoryClient,
    state: JointTrajectoryControllerState,
    positions: &[f64],
    duration: std::time::Duration,
) -> Result<msg::trajectory_msgs::JointTrajectory, arci::Error> {
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

    let point_with_full_positions = msg::trajectory_msgs::JointTrajectoryPoint {
        positions: full_positions,
        // add zero velocity to use cubic interpolation in trajectory_controller
        velocities: vec![0.0; full_dof],
        time_from_start: rosrust::Duration::from_nanos(duration.as_nanos() as i64),
        ..Default::default()
    };
    Ok(msg::trajectory_msgs::JointTrajectory {
        joint_names: full_names,
        points: vec![point_with_full_positions],
        ..Default::default()
    })
}

pub fn create_joint_trajectory_message_for_send_joint_trajectory(
    client: &dyn JointTrajectoryClient,
    state: JointTrajectoryControllerState,
    trajectory: &[TrajectoryPoint],
) -> Result<msg::trajectory_msgs::JointTrajectory, arci::Error> {
    // TODO: cache index map and use it
    let current_full_positions = state.actual.positions;
    let full_names = state.joint_names;
    let full_dof = current_full_positions.len();

    Ok(msg::trajectory_msgs::JointTrajectory {
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
                msg::trajectory_msgs::JointTrajectoryPoint {
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
                    time_from_start: rosrust::Duration {
                        sec: tp.time_from_start.as_secs() as i32,
                        nsec: tp.time_from_start.subsec_nanos() as i32,
                    },
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
    trajectory_publisher: rosrust::Publisher<RosJointTrajectory>,
    joint_state_subscriber_handler: SubscriberHandler<JointTrajectoryControllerState>,
    complete_condition: Box<dyn CompleteCondition>,
}

impl RosControlClient {
    pub fn new(joint_names: Vec<String>, controller_name: &str) -> Self {
        let joint_state_topic_name = format!("{}/state", controller_name);
        let joint_state_subscriber_handler = SubscriberHandler::new(&joint_state_topic_name, 1);
        joint_state_subscriber_handler.wait_message(100);

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
            joint_state_subscriber_handler,
            complete_condition: Box::new(TotalJointDiffCondition::default()),
        }
    }
    pub fn get_joint_state(&self) -> Result<JointTrajectoryControllerState, arci::Error> {
        self.joint_state_subscriber_handler
            .get()?
            .ok_or_else(|| arci::Error::Other(Error::NoJointStateAvailable.into()))
    }
}

#[async_trait]
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

    async fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<(), arci::Error> {
        let traj = create_joint_trajectory_message_for_send_joint_positions(
            self,
            self.get_joint_state()?,
            &positions,
            duration,
        )?;
        self.trajectory_publisher.send(traj).unwrap();
        self.complete_condition.wait(self, &positions)
    }
    async fn send_joint_trajectory(
        &self,
        trajectory: Vec<TrajectoryPoint>,
    ) -> Result<(), arci::Error> {
        let traj = create_joint_trajectory_message_for_send_joint_trajectory(
            self,
            self.get_joint_state()?,
            &trajectory,
        )?;
        self.trajectory_publisher.send(traj).unwrap();
        self.complete_condition
            .wait(self, &trajectory.last().unwrap().positions)
    }
}

impl SetCompleteCondition for RosControlClient {
    fn set_complete_condition(&mut self, condition: Box<dyn CompleteCondition>) {
        self.complete_condition = condition;
    }
}
