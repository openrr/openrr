use std::{path::PathBuf, sync::Arc};

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
use tracing::{error, info};

use crate::ControlNode;

const MODE: &str = "command";

pub struct RobotCommandExecutor<S>
where
    S: Speaker,
{
    base_path: PathBuf,
    commands: Vec<RobotCommandConfig>,
    robot_client: Arc<ArcRobotClient>,
    speaker: S,
    submode: String,
    command_index: usize,
    is_trigger_holding: bool,
    is_sending: bool,
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
            let submode = commands[0].name.clone();
            Some(Self {
                base_path,
                commands,
                robot_client,
                speaker,
                submode,
                command_index: 0,
                is_trigger_holding: false,
                is_sending: false,
            })
        }
    }
}

#[async_trait]
impl<S> ControlNode for RobotCommandExecutor<S>
where
    S: Speaker,
{
    fn set_event(&mut self, event: arci::gamepad::GamepadEvent) {
        match event {
            GamepadEvent::ButtonPressed(Button::East) => {
                self.command_index = (self.command_index + 1) % self.commands.len();
                let command = &self.commands[self.command_index];
                self.submode = command.name.clone();
                // do not wait
                let _ = self
                    .speaker
                    .speak(&format!("{} {}", MODE, self.submode()))
                    .unwrap();
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
            _ => {}
        }
    }

    async fn proc(&self) {
        if self.is_trigger_holding && self.is_sending {
            let command = &self.commands[self.command_index];
            let executor = openrr_command::RobotCommandExecutor {};
            match resolve_relative_path(&self.base_path, command.file_path.clone()) {
                Ok(path) => {
                    match load_command_file_and_filter(path) {
                        Ok(commands) => {
                            let commands_len = commands.len() as f64;
                            for (i, command) in commands.iter().enumerate() {
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
    }

    fn mode(&self) -> &str {
        &MODE
    }

    fn submode(&self) -> &str {
        &self.submode
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RobotCommandConfig {
    pub name: String,
    pub file_path: String,
}
