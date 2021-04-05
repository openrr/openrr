use crate::msg;
use arci::*;
use async_trait::async_trait;
use std::sync::{Arc, Mutex};

pub struct RosRobotClient {
    joint_names: Vec<String>,
    trajectory_publisher: Option<rosrust::Publisher<msg::trajectory_msgs::JointTrajectory>>,
    _joint_state_subscriber: rosrust::Subscriber,
    joint_state_message: Arc<Mutex<msg::sensor_msgs::JointState>>,
    complete_condition: Box<dyn CompleteCondition>,
}

impl From<TrajectoryPoint> for msg::trajectory_msgs::JointTrajectoryPoint {
    fn from(tp: TrajectoryPoint) -> Self {
        let mut message = msg::trajectory_msgs::JointTrajectoryPoint {
            positions: tp.positions,
            ..Default::default()
        };
        message.time_from_start.sec = tp.time_from_start.as_secs() as i32;
        message.time_from_start.nsec = tp.time_from_start.subsec_nanos() as i32;
        message
    }
}

impl RosRobotClient {
    pub fn new(
        joint_names: Vec<String>,
        joint_state_topic_name: &str,
        trajectory_topic_name: &str,
    ) -> Self {
        let joint_state_message = Arc::new(Mutex::new(msg::sensor_msgs::JointState::default()));
        let joint_state_message_for_sub = joint_state_message.clone();
        let _joint_state_subscriber = rosrust::subscribe(
            joint_state_topic_name,
            1,
            move |joint_state: msg::sensor_msgs::JointState| {
                let mut aaa = joint_state_message_for_sub.lock().unwrap();
                *aaa = joint_state;
            },
        )
        .unwrap();
        while joint_state_message.lock().unwrap().name.is_empty() {
            rosrust::ros_info!("waiting joint state publisher");
            std::thread::sleep(std::time::Duration::from_millis(100));
        }

        let trajectory_publisher = if trajectory_topic_name.is_empty() {
            None
        } else {
            let publisher = rosrust::publish(trajectory_topic_name, 1).unwrap();

            let rate = rosrust::rate(10.0);
            while rosrust::is_ok() && publisher.subscriber_count() == 0 {
                rosrust::ros_info!("waiting trajectory subscriber");
                rate.sleep();
            }
            Some(publisher)
        };

        Self {
            joint_names,
            trajectory_publisher,
            _joint_state_subscriber,
            joint_state_message,
            complete_condition: Box::new(TotalJointDiffCondition::default()),
        }
    }
}

#[async_trait]
impl JointTrajectoryClient for RosRobotClient {
    fn joint_names(&self) -> &[String] {
        &self.joint_names
    }
    fn current_joint_positions(&self) -> Result<Vec<f64>> {
        let message = self.joint_state_message.lock().unwrap();
        Ok(message.position.clone())
    }

    async fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<()> {
        if let Some(ref publisher) = self.trajectory_publisher {
            if self.joint_names.len() != positions.len() {
                return Err(arci::Error::LengthMismatch {
                    model: self.joint_names.len(),
                    input: positions.len(),
                });
            }
            let point = msg::trajectory_msgs::JointTrajectoryPoint {
                positions: positions.to_vec(),
                time_from_start: rosrust::Duration::from_nanos(duration.as_nanos() as i64),
                ..Default::default()
            };
            let traj = msg::trajectory_msgs::JointTrajectory {
                joint_names: self.joint_names.clone(),
                points: vec![point],
                ..Default::default()
            };
            publisher.send(traj).unwrap();
            self.complete_condition
                .wait(self, &positions, duration.as_secs_f64())?;
        }
        Ok(())
    }
    async fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<()> {
        if let Some(ref publisher) = self.trajectory_publisher {
            let traj = msg::trajectory_msgs::JointTrajectory {
                joint_names: self.joint_names.clone(),
                points: trajectory.iter().map(|t| (*t).clone().into()).collect(),
                ..Default::default()
            };
            publisher.send(traj).unwrap();
            self.complete_condition.wait(
                self,
                &trajectory.last().unwrap().positions,
                trajectory.last().unwrap().time_from_start.as_secs_f64(),
            )?;
        }
        Ok(())
    }
}

impl SetCompleteCondition for RosRobotClient {
    fn set_complete_condition(&mut self, condition: Box<dyn CompleteCondition>) {
        self.complete_condition = condition;
    }
}
