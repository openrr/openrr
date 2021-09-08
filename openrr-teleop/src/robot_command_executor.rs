use std::{
    path::PathBuf,
    sync::{Arc, Mutex},
};

use arci::{
    gamepad::{Button, GamepadEvent},
    Speaker,
};
use async_trait::async_trait;
use openrr_client::{resolve_relative_path, ArcRobotClient};
use openrr_command::{load_command_file_and_filter, RobotCommand};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use structopt::StructOpt;
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
        if let Some(submode) = self.inner.lock().unwrap().handle_event(event) {
            // do not wait
            let _ = self
                .speaker
                .speak(&format!("{} {}", MODE, submode))
                .unwrap();
        }
    }

    async fn proc(&self) {
        let command = {
            let inner = self.inner.lock().unwrap();
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
                            if !self.inner.lock().unwrap().is_trigger_holding {
                                warn!("Remaining commands are canceled.");
                                return;
                            }
                            let command_parsed_iter = command.split_whitespace();
                            // Parse the command
                            let read_opt = RobotCommand::from_iter(command_parsed_iter);
                            // Execute the parsed command
                            info!("Executing {} {}/{}", command, i, commands_len);

                            match executor.execute(&self.robot_client, &read_opt).await {
                                Ok(_) => {
                                    info!("finished command {}", command);
                                }
                                Err(e) => {
                                    error!("failed command {} {:?}", command, e);
                                    break;
                                }
                            }
                        }
                    }
                    Err(e) => {
                        error!("failed to load file {:?}", e);
                    }
                }
            }
            Err(e) => {
                error!("failed to resolve_relative_path {:?}", e);
            }
        }
    }

    fn mode(&self) -> &str {
        MODE
    }

    fn submode(&self) -> String {
        self.inner.lock().unwrap().submode.to_owned()
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RobotCommandConfig {
    pub name: String,
    pub file_path: String,
}
