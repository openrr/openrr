use std::{path::PathBuf, sync::Arc};

use arci::{
    gamepad::{Button, GamepadEvent},
    Speaker,
};
use async_trait::async_trait;
use clap::Parser;
use openrr_client::{resolve_relative_path, ArcRobotClient};
use openrr_command::{load_command_file_and_filter, RobotCommand};
use parking_lot::Mutex;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::{error, info, warn};

use crate::ControlNode;

const MODE: &str = "command";

struct RobotCommandExecutorInner {
    commands: Vec<RobotCommandConfig>,
    submode: String,
    command_index: usize,
    is_trigger_holding: bool,
    is_sending: bool,
}

impl RobotCommandExecutorInner {
    fn new(commands: Vec<RobotCommandConfig>) -> Self {
        let submode = commands[0].name.clone();
        Self {
            commands,
            submode,
            command_index: 0,
            is_trigger_holding: false,
            is_sending: false,
        }
    }

    fn handle_event(&mut self, event: arci::gamepad::GamepadEvent) -> Option<&str> {
        match event {
            GamepadEvent::ButtonPressed(Button::East) => {
                self.command_index = (self.command_index + 1) % self.commands.len();
                let command = &self.commands[self.command_index];
                self.submode = command.name.clone();
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

    fn get_command(&self) -> &RobotCommandConfig {
        &self.commands[self.command_index]
    }
}

pub struct RobotCommandExecutor<S>
where
    S: Speaker,
{
    base_path: PathBuf,
    robot_client: Arc<ArcRobotClient>,
    speaker: S,
    inner: Mutex<RobotCommandExecutorInner>,
}

impl<S> RobotCommandExecutor<S>
where
    S: Speaker,
{
    pub fn new(
        base_path: PathBuf,
        commands: Vec<RobotCommandConfig>,
        robot_client: Arc<ArcRobotClient>,
        speaker: S,
    ) -> Option<Self> {
        if commands.is_empty() {
            None
        } else {
            Some(Self {
                base_path,
                robot_client,
                speaker,
                inner: Mutex::new(RobotCommandExecutorInner::new(commands)),
            })
        }
    }
}

#[async_trait]
impl<S> ControlNode for RobotCommandExecutor<S>
where
    S: Speaker,
{
    fn handle_event(&self, event: arci::gamepad::GamepadEvent) {
        if let Some(submode) = self.inner.lock().handle_event(event) {
            // do not wait
            let _ = self.speaker.speak(&format!("{MODE} {submode}")).unwrap();
        }
    }

    async fn proc(&self) {
        let command = {
            let inner = self.inner.lock();
            if inner.is_trigger_holding && inner.is_sending {
                inner.get_command().clone()
            } else {
                return;
            }
        };
        let executor = openrr_command::RobotCommandExecutor {};
        match resolve_relative_path(&self.base_path, command.file_path.clone()) {
            Ok(path) => {
                match load_command_file_and_filter(path) {
                    Ok(commands) => {
                        let commands_len = commands.len() as f64;
                        for (i, command) in commands.iter().enumerate() {
                            if !self.inner.lock().is_trigger_holding {
                                warn!("Remaining commands are canceled.");
                                return;
                            }
                            let command_parsed_iter = command.split_whitespace();
                            // Parse the command
                            let read_opt = RobotCommand::parse_from(command_parsed_iter);
                            // Execute the parsed command
                            info!("Executing {command} {i}/{commands_len}");

                            match executor.execute(&self.robot_client, &read_opt).await {
                                Ok(_) => {
                                    info!("finished command {command}");
                                }
                                Err(e) => {
                                    error!("failed command {command} {e:?}");
                                    break;
                                }
                            }
                        }
                    }
                    Err(e) => {
                        error!("failed to load file {e:?}");
                    }
                }
            }
            Err(e) => {
                error!("failed to resolve_relative_path {e:?}");
            }
        }
    }

    fn mode(&self) -> &str {
        MODE
    }

    fn submode(&self) -> String {
        self.inner.lock().submode.to_owned()
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RobotCommandConfig {
    pub name: String,
    pub file_path: String,
}

#[cfg(test)]
mod test {
    use std::collections::HashMap;

    use arci::{
        DummyJointTrajectoryClient, DummyLocalization, DummyMoveBase, DummyNavigation,
        DummySpeaker, JointTrajectoryClient, Localization, MoveBase, Navigation,
    };
    use openrr_client::RobotClient;

    use super::*;

    #[test]
    fn test_robot_command_executor_inner() {
        let commands = vec![
            RobotCommandConfig {
                name: String::from("name0"),
                file_path: String::from("path0"),
            },
            RobotCommandConfig {
                name: String::from("name1"),
                file_path: String::from("path1"),
            },
        ];
        let mut inner = RobotCommandExecutorInner::new(commands);

        assert_eq!(inner.get_command().name, "name0");
        assert_eq!(inner.get_command().file_path, "path0");

        // Changed submode
        inner.handle_event(GamepadEvent::ButtonPressed(Button::East));
        assert_eq!(inner.submode, "name1");
        assert_eq!(inner.command_index, 1);

        // Case that enable switch is on
        inner.handle_event(GamepadEvent::ButtonPressed(Button::RightTrigger2));
        assert!(inner.is_trigger_holding);
        assert!(!inner.is_sending);

        // Case that enable switch becomes off
        inner.handle_event(GamepadEvent::ButtonReleased(Button::RightTrigger2));
        assert!(!inner.is_trigger_holding);
        assert!(!inner.is_sending);

        // Send command
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
    fn test_robot_command_executor_new() {
        let robot_client = Arc::new(
            RobotClient::new(
                toml::from_str("urdf_path = \"path\"").unwrap(),
                {
                    HashMap::from([(
                        String::from("arm"),
                        Arc::new(DummyJointTrajectoryClient::new(vec![String::from(
                            "test_joint1",
                        )])) as Arc<dyn JointTrajectoryClient>,
                    )])
                },
                {
                    HashMap::from([(
                        String::from("speaker"),
                        Arc::new(DummySpeaker::new()) as Arc<dyn Speaker>,
                    )])
                },
                Some(Arc::new(DummyLocalization::new()) as Arc<dyn Localization>),
                Some(Arc::new(DummyMoveBase::new()) as Arc<dyn MoveBase>),
                Some(Arc::new(DummyNavigation::new()) as Arc<dyn Navigation>),
            )
            .unwrap(),
        );

        let robot_command_executor = RobotCommandExecutor::new(
            PathBuf::from("path"),
            vec![RobotCommandConfig {
                name: String::from("name"),
                file_path: String::from("file_path"),
            }],
            robot_client.clone(),
            DummySpeaker::new(),
        )
        .unwrap();

        assert_eq!(robot_command_executor.base_path, PathBuf::from("path"));
        assert_eq!(
            robot_command_executor.inner.lock().commands[0].name,
            String::from("name")
        );
        assert_eq!(
            robot_command_executor.inner.lock().commands[0].file_path,
            String::from("file_path")
        );

        assert!(RobotCommandExecutor::new(
            PathBuf::from("path"),
            vec![],
            robot_client,
            DummySpeaker::new()
        )
        .is_none());
    }
}
