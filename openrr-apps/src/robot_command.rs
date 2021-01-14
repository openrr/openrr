use crate::Error as OpenrrAppsError;
use async_recursion::async_recursion;
use log::info;
use openrr_client::isometry;
use std::{
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::PathBuf,
};
use structopt::StructOpt;

use crate::{RobotClient, RobotConfig};

#[derive(StructOpt, Debug)]
#[structopt(name = "robot_command", about = "An openrr command line tool.")]
pub struct RobotCommand {
    #[structopt(short, long, parse(from_os_str))]
    config_path: Option<PathBuf>,
    #[structopt(subcommand)]
    sub_command: RobotSubCommand,
}

fn parse_joints<T, U>(s: &str) -> Result<(T, U), Box<dyn Error>>
where
    T: std::str::FromStr,
    T::Err: Error + 'static,
    U: std::str::FromStr,
    U::Err: Error + 'static,
{
    let pos = s
        .find('=')
        .ok_or_else(|| format!("invalid KEY=value: no `=` found in `{}`", s))?;
    Ok((s[..pos].parse()?, s[pos + 1..].parse()?))
}

#[derive(StructOpt, Debug)]
#[structopt(rename_all = "snake_case")]
pub enum RobotSubCommand {
    /// Send joint positions.
    Joints {
        name: String,
        #[structopt(short, long, default_value = "3.0")]
        duration: f64,
        /// Interpolate target in cartesian space.
        /// If you use this flag, joint values are not used as references but used in forward kinematics.
        #[structopt(name = "interpolate", short, long)]
        use_interpolation: bool,
        #[structopt(short, parse(try_from_str=parse_joints))]
        joint: Vec<(usize, f64)>,
    },
    /// Move with ik
    Ik {
        name: String,
        #[structopt(short, long)]
        x: Option<f64>,
        #[structopt(short, long)]
        y: Option<f64>,
        #[structopt(short, long)]
        z: Option<f64>,
        #[structopt(long)]
        yaw: Option<f64>,
        #[structopt(short, long)]
        pitch: Option<f64>,
        #[structopt(short, long)]
        roll: Option<f64>,
        #[structopt(short, long, default_value = "3.0")]
        duration: f64,
        /// Interpolate target in cartesian space.
        #[structopt(name = "interpolate", short, long)]
        use_interpolation: bool,
        #[structopt(name = "local", short, long)]
        is_local: bool,
    },
    /// Get joint positions and end pose if applicable.
    Get { name: String },
    /// Load commands from file and execute them.
    Load {
        #[structopt(parse(from_os_str))]
        command_file_path: PathBuf,
    },
    /// List available clients.
    List {},
    /// Speak text message.
    Speak { message: Vec<String> },
}

impl RobotCommand {
    pub async fn execute(&self) -> Result<(), OpenrrAppsError> {
        if let Some(config_path) = &self.config_path {
            let config = RobotConfig::try_new(config_path)?;
            let client = RobotClient::try_new(config)?;
            self.execute_sub_command(&client, &self).await
        } else {
            return Err(OpenrrAppsError::NoConfigPath);
        }
    }

    #[async_recursion]
    pub async fn execute_sub_command(
        &self,
        client: &RobotClient,
        command: &RobotCommand,
    ) -> Result<(), OpenrrAppsError> {
        match &command.sub_command {
            RobotSubCommand::Joints {
                name,
                duration,
                use_interpolation,
                joint,
            } => {
                let mut positions = client.current_joint_positions(name).await?;

                let mut should_send = false;
                for (index, position) in joint {
                    if *index < positions.len() {
                        should_send = true;
                        positions[*index] = *position;
                    }
                }
                if !should_send {
                    return Ok(());
                }
                if *use_interpolation {
                    client
                        .send_joint_positions_with_pose_interpolation(name, &positions, *duration)
                        .await?;
                } else {
                    client
                        .send_joint_positions(name, &positions, *duration)
                        .await?;
                }
            }
            RobotSubCommand::Ik {
                name,
                x,
                y,
                z,
                yaw,
                pitch,
                roll,
                duration,
                use_interpolation,
                is_local,
            } => {
                if !client.is_ik_client(name) {
                    return Err(OpenrrAppsError::NoIkClient(name.clone()));
                }
                let mut should_send = false;
                let current_pose = client.current_end_transform(name).await?;
                let target_pose = [
                    if let Some(x) = x {
                        should_send = true;
                        *x
                    } else if *is_local {
                        0.0
                    } else {
                        current_pose.translation.x
                    },
                    if let Some(y) = y {
                        should_send = true;
                        *y
                    } else if *is_local {
                        0.0
                    } else {
                        current_pose.translation.y
                    },
                    if let Some(z) = z {
                        should_send = true;
                        *z
                    } else if *is_local {
                        0.0
                    } else {
                        current_pose.translation.z
                    },
                    if let Some(roll) = roll {
                        should_send = true;
                        *roll
                    } else if *is_local {
                        0.0
                    } else {
                        current_pose.rotation.euler_angles().0
                    },
                    if let Some(pitch) = pitch {
                        should_send = true;
                        *pitch
                    } else if *is_local {
                        0.0
                    } else {
                        current_pose.rotation.euler_angles().1
                    },
                    if let Some(yaw) = yaw {
                        should_send = true;
                        *yaw
                    } else if *is_local {
                        0.0
                    } else {
                        current_pose.rotation.euler_angles().2
                    },
                ];
                if !should_send {
                    return Ok(());
                }
                let target_pose = isometry(
                    target_pose[0],
                    target_pose[1],
                    target_pose[2],
                    target_pose[3],
                    target_pose[4],
                    target_pose[5],
                );

                let target_pose = if *is_local {
                    client.transform(name, &target_pose).await?
                } else {
                    target_pose
                };
                if *use_interpolation {
                    client
                        .move_ik_with_interpolation(name, &target_pose, *duration)
                        .await?
                } else {
                    client.move_ik(name, &target_pose, *duration).await?
                }
            }
            RobotSubCommand::Get { name } => {
                info!(
                    "Joint positions : {:?}",
                    client.current_joint_positions(name).await?
                );
                if client.is_ik_client(name) {
                    let pose = client.current_end_transform(name).await?;
                    info!("End pose");
                    info!(" translation = {:?}", pose.translation.vector.data);
                    info!(" rotation = {:?}", pose.rotation.euler_angles());
                }
            }
            RobotSubCommand::Load { command_file_path } => {
                for command in load_command_file_and_filter(command_file_path.clone())? {
                    let command_parsed_iter = command.split_whitespace();
                    // Parse the command
                    let read_opt = RobotCommand::from_iter(command_parsed_iter);
                    // Execute the parsed command
                    info!("Executing {}", command);
                    self.execute_sub_command(&client, &read_opt).await?;
                }
            }
            RobotSubCommand::List {} => {
                client.list_clients();
            }
            RobotSubCommand::Speak { message } => {
                // TODO: Parse quotations and comments
                // Currently '"Foo bar" # hoge' is parsed as message in below command.
                // 'openrr_apps_robot_command speak "Foo bar" # hoge'
                client.speak(&message.join(" "));
            }
        }
        Ok(())
    }
}

pub fn load_command_file_and_filter(file_path: PathBuf) -> Result<Vec<String>, OpenrrAppsError> {
    let file = File::open(&file_path)
        .map_err(|e| OpenrrAppsError::CommandFileOpenFailure(file_path, e.to_string()))?;
    let buf = BufReader::new(file);
    Ok(buf
        .lines()
        .map(|line| line.expect("Could not parse line"))
        .filter(|command| {
            let command_parsed_iter = command.split_whitespace();
            // Ignore empty lines and comment lines
            command_parsed_iter.clone().count() > 0
                && command_parsed_iter.clone().next().unwrap().find('#') == None
        })
        .collect())
}
