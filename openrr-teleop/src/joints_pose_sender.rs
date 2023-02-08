use std::{collections::HashMap, time::Duration};

use arci::{
    gamepad::{Button, GamepadEvent},
    JointTrajectoryClient, Speaker,
};
use async_trait::async_trait;
use openrr_client::JointsPose;
use parking_lot::Mutex;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::ControlMode;

struct JointsPoseSenderInner {
    joints_poses: Vec<JointsPose>,
    submode: String,
    pose_index: usize,
    is_trigger_holding: bool,
    is_sending: bool,
}

impl JointsPoseSenderInner {
    fn new(joints_poses: Vec<JointsPose>) -> Self {
        Self {
            submode: format!(
                " {} {}",
                joints_poses[0].client_name, joints_poses[0].pose_name
            ),
            joints_poses,
            pose_index: 0,
            is_trigger_holding: false,
            is_sending: false,
        }
    }

    fn handle_event(&mut self, event: arci::gamepad::GamepadEvent) -> Option<&str> {
        match event {
            GamepadEvent::ButtonPressed(Button::East) => {
                self.pose_index = (self.pose_index + 1) % self.joints_poses.len();
                let joints_pose = &self.joints_poses[self.pose_index];
                self.submode = format!(" {} {}", joints_pose.client_name, joints_pose.pose_name);
                return Some(&self.submode);
            }
            GamepadEvent::ButtonPressed(Button::RightTrigger2) => {
                self.is_trigger_holding = true;
            }
            GamepadEvent::ButtonReleased(Button::RightTrigger2) => {
                self.is_trigger_holding = false;
                self.is_sending = false;
            }
            GamepadEvent::ButtonPressed(Button::West) => {
                self.is_sending = true;
            }
            GamepadEvent::ButtonReleased(Button::West) => {
                self.is_sending = false;
            }
            GamepadEvent::Disconnected => {
                self.is_trigger_holding = false;
                self.is_sending = false;
            }
            _ => {}
        }
        None
    }

    fn get_target_name_positions(&self) -> (String, Vec<f64>) {
        let joints_pose = &self.joints_poses[self.pose_index];
        (
            joints_pose.client_name.to_owned(),
            joints_pose.positions.to_owned(),
        )
    }
}
pub struct JointsPoseSender<S, J>
where
    S: Speaker,
    J: JointTrajectoryClient,
{
    mode: String,
    joint_trajectory_clients: HashMap<String, J>,
    speaker: S,
    duration: Duration,
    inner: Mutex<JointsPoseSenderInner>,
}

impl<S, J> JointsPoseSender<S, J>
where
    S: Speaker,
    J: JointTrajectoryClient,
{
    pub fn new(
        mode: String,
        joints_poses: Vec<JointsPose>,
        joint_trajectory_clients: HashMap<String, J>,
        speaker: S,
        duration: Duration,
    ) -> Self {
        Self {
            mode,
            joint_trajectory_clients,
            speaker,
            duration,
            inner: Mutex::new(JointsPoseSenderInner::new(joints_poses)),
        }
    }

    pub fn new_from_config(
        config: JointsPoseSenderConfig,
        joints_poses: Vec<JointsPose>,
        joint_trajectory_clients: HashMap<String, J>,
        speaker: S,
    ) -> Self {
        Self::new(
            config.mode,
            joints_poses,
            joint_trajectory_clients,
            speaker,
            Duration::from_secs_f64(config.duration_secs),
        )
    }
}

#[async_trait]
impl<S, J> ControlMode for JointsPoseSender<S, J>
where
    S: Speaker,
    J: JointTrajectoryClient,
{
    fn handle_event(&self, event: arci::gamepad::GamepadEvent) {
        if let Some(submode) = self.inner.lock().handle_event(event) {
            // do not wait
            drop(
                self.speaker
                    .speak(&format!("{}{submode}", self.mode))
                    .unwrap(),
            );
        }
    }

    async fn proc(&self) {
        let inner = self.inner.lock();
        let (name, target) = inner.get_target_name_positions();
        let client = self.joint_trajectory_clients.get(&name).unwrap();
        drop(
            client
                .send_joint_positions(
                    if inner.is_sending && inner.is_trigger_holding {
                        target
                    } else {
                        client.current_joint_positions().unwrap()
                    },
                    self.duration,
                )
                .unwrap(),
        );
    }

    fn mode(&self) -> &str {
        &self.mode
    }

    fn submode(&self) -> String {
        self.inner.lock().submode.to_owned()
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct JointsPoseSenderConfig {
    #[serde(default = "default_mode")]
    pub mode: String,
    #[serde(default = "default_duration_secs")]
    pub duration_secs: f64,
}

fn default_mode() -> String {
    "pose".to_string()
}

fn default_duration_secs() -> f64 {
    2.0
}

#[cfg(test)]
mod test {
    use arci::DummyJointTrajectoryClient;
    use assert_approx_eq::*;
    use openrr_client::PrintSpeaker;

    use super::*;

    #[test]
    fn test_joints_pose_sender_inner() {
        let joints_poses = vec![
            JointsPose {
                pose_name: String::from("pose0"),
                client_name: String::from("client0"),
                positions: vec![1.2, 3.4, 5.6],
            },
            JointsPose {
                pose_name: String::from("pose1"),
                client_name: String::from("client1"),
                positions: vec![7.8, 9.1, 2.3],
            },
        ];
        let mut inner = JointsPoseSenderInner::new(joints_poses);

        assert_eq!(inner.get_target_name_positions().0, String::from("client0"));
        assert_approx_eq!(inner.get_target_name_positions().1[0], 1.2f64);
        assert_approx_eq!(inner.get_target_name_positions().1[1], 3.4f64);
        assert_approx_eq!(inner.get_target_name_positions().1[2], 5.6f64);

        // Change submode
        inner.handle_event(GamepadEvent::ButtonPressed(Button::East));
        assert_eq!(inner.submode, String::from(" client1 pose1"));
        assert_eq!(inner.pose_index, 1);

        // Case that enable switch is on
        inner.handle_event(GamepadEvent::ButtonPressed(Button::RightTrigger2));
        assert!(inner.is_trigger_holding);
        assert!(!inner.is_sending);

        // Case that enable switch becomes off
        inner.handle_event(GamepadEvent::ButtonReleased(Button::RightTrigger2));
        assert!(!inner.is_trigger_holding);
        assert!(!inner.is_sending);

        // Send pose
        inner.handle_event(GamepadEvent::ButtonPressed(Button::West));
        assert!(inner.is_sending);

        // Stop send command
        inner.handle_event(GamepadEvent::ButtonReleased(Button::West));
        assert!(!inner.is_sending);

        // Disconnected
        inner.handle_event(GamepadEvent::Disconnected);
        assert!(!inner.is_trigger_holding);
        assert!(!inner.is_sending);
    }

    #[test]
    fn test_joints_pose_sender() {
        let mode = String::from("test_mode");
        let joints_poses = vec![JointsPose {
            pose_name: String::from("pose0"),
            client_name: String::from("client0"),
            positions: vec![1.2, 3.4, 5.6],
        }];
        let joint_names = vec![String::from("test_joint1")];
        let joint_trajectory_clients = HashMap::from([(
            String::from("client0"),
            DummyJointTrajectoryClient::new(joint_names.clone()),
        )]);
        let speaker = PrintSpeaker::new();
        let duration = Duration::from_millis(5);

        let inner = Mutex::new(JointsPoseSenderInner::new(joints_poses.clone()));

        let joints_pose_sender = JointsPoseSender::new(
            mode.clone(),
            joints_poses,
            joint_trajectory_clients,
            speaker,
            duration,
        );

        // Test for generate joints_pose_sender
        assert_eq!(joints_pose_sender.mode, mode);
        assert_eq!(
            format!("{:?}", DummyJointTrajectoryClient::new(joint_names)),
            format!(
                "{:?}",
                joints_pose_sender.joint_trajectory_clients["client0"]
            )
        );
        assert!(duration.eq(&joints_pose_sender.duration));
        assert_eq!(
            format!("{:?}", joints_pose_sender.inner.lock().joints_poses),
            format!("{:?}", inner.lock().joints_poses)
        );

        // Test fot getting mode of joints_pose_sender
        let joints_pose_sender_mode = joints_pose_sender.mode();
        assert_eq!(joints_pose_sender_mode, mode);
        let submode = joints_pose_sender.submode();
        assert_eq!(joints_pose_sender.inner.lock().submode, submode);
    }

    #[tokio::test]
    async fn test_joints_pose_sender_proc() {
        let joints_poses = vec![JointsPose {
            pose_name: String::from("pose0"),
            client_name: String::from("client0"),
            positions: vec![1.2, 3.4, 5.6],
        }];
        let joint_names = vec![
            String::from("joint1"),
            String::from("joint2"),
            String::from("joint3"),
        ];

        let joints_pose_sender = JointsPoseSender {
            mode: String::from("test_mode"),
            joint_trajectory_clients: HashMap::from([(
                String::from("client0"),
                DummyJointTrajectoryClient::new(joint_names.clone()),
            )]),
            speaker: PrintSpeaker::new(),
            duration: Duration::from_millis(5),
            inner: Mutex::new(JointsPoseSenderInner {
                joints_poses: joints_poses.clone(),
                submode: String::from("submode"),
                pose_index: 0,
                is_trigger_holding: false,
                is_sending: false,
            }),
        };
        joints_pose_sender.proc().await;

        let current_state = joints_pose_trajectory_client_current_state(&joints_pose_sender);

        assert_approx_eq!(current_state[0], 0.0);
        assert_approx_eq!(current_state[1], 0.0);
        assert_approx_eq!(current_state[2], 0.0);

        let joints_pose_sender = JointsPoseSender {
            mode: String::from("test_mode"),
            joint_trajectory_clients: HashMap::from([(
                String::from("client0"),
                DummyJointTrajectoryClient::new(joint_names.clone()),
            )]),
            speaker: PrintSpeaker::new(),
            duration: Duration::from_millis(5),
            inner: Mutex::new(JointsPoseSenderInner {
                joints_poses: joints_poses.clone(),
                submode: String::from("submode"),
                pose_index: 0,
                is_trigger_holding: true,
                is_sending: true,
            }),
        };
        joints_pose_sender.proc().await;

        let current_state = joints_pose_trajectory_client_current_state(&joints_pose_sender);

        assert_approx_eq!(current_state[0], 1.2);
        assert_approx_eq!(current_state[1], 3.4);
        assert_approx_eq!(current_state[2], 5.6);
    }

    fn joints_pose_trajectory_client_current_state(
        joints_pose_sender: &JointsPoseSender<PrintSpeaker, DummyJointTrajectoryClient>,
    ) -> Vec<f64> {
        let inner = joints_pose_sender.inner.lock();
        let (name, target) = inner.get_target_name_positions();

        let client = joints_pose_sender
            .joint_trajectory_clients
            .get(&name)
            .unwrap();
        drop(
            client
                .send_joint_positions(
                    if inner.is_sending && inner.is_trigger_holding {
                        target
                    } else {
                        client.current_joint_positions().unwrap()
                    },
                    joints_pose_sender.duration,
                )
                .unwrap(),
        );

        joints_pose_sender.joint_trajectory_clients[&name]
            .current_joint_positions()
            .unwrap()
    }

    #[test]
    fn test_default_mode() {
        assert_eq!(default_mode(), "pose".to_string());
    }

    #[test]
    fn test_default_duration_secs() {
        let def = default_duration_secs();
        assert_approx_eq!(def, 2.0f64);
    }
}
