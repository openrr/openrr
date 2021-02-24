use crate::Error as OpenrrCommandError;
use arci::{BaseVelocity, MoveBase, Navigation, Speaker};
use async_recursion::async_recursion;
use k::nalgebra::{Isometry2, Vector2};
use openrr_client::{isometry, BoxRobotClient};
use std::{
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::PathBuf,
    process::Command,
    thread::sleep,
    time::{Duration, Instant},
};
use structopt::StructOpt;
use tracing::{error, info};

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
pub enum RobotCommand {
    /// Send joint positions.
    SendJoints {
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
    /// Send predefined joint positions.
    SendJointsPose {
        name: String,
        pose_name: String,
        #[structopt(short, long, default_value = "3.0")]
        duration: f64,
    },
    /// Move with ik
    MoveIk {
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
    GetState { name: String },
    /// Load commands from file and execute them.
    LoadCommands {
        #[structopt(parse(from_os_str))]
        command_file_path: PathBuf,
    },
    /// List available clients.
    List,
    /// Speak text message.
    Speak { message: Vec<String> },
    /// Execute an external command.
    ExecuteCommand { command: Vec<String> },
    /// Get navigation current pose.
    GetNavigationCurrentPose,
    /// Send navigation goal pose.
    SendNavigationGoal {
        x: f64,
        y: f64,
        yaw: f64,
        #[structopt(short, long, default_value = "map")]
        frame_id: String,
        #[structopt(short, long, default_value = "100.0")]
        timeout_secs: f64,
    },
    /// Cancel navigation gaol.
    CancelNavigationGoal,
    /// Send base velocity.
    SendBaseVelocity {
        x: f64,
        y: f64,
        theta: f64,
        #[structopt(short, long, default_value = "1.0")]
        duration_secs: f64,
    },
}

pub struct RobotCommandExecutor {}

impl RobotCommandExecutor {
    #[async_recursion]
    pub async fn execute(
        &self,
        client: &BoxRobotClient,
        command: &RobotCommand,
    ) -> Result<(), OpenrrCommandError> {
        match &command {
            RobotCommand::SendJoints {
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
            RobotCommand::SendJointsPose {
                name,
                pose_name,
                duration,
            } => {
                client.send_joints_pose(name, pose_name, *duration).await?;
            }
            RobotCommand::MoveIk {
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
                    return Err(OpenrrCommandError::NoIkClient(name.clone()));
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
            RobotCommand::GetState { name } => {
                println!(
                    "Joint positions : {:?}",
                    client.current_joint_positions(name).await?
                );
                if client.is_ik_client(name) {
                    let pose = client.current_end_transform(name).await?;
                    println!("End pose");
                    println!(" translation = {:?}", pose.translation.vector.data);
                    println!(" rotation = {:?}", pose.rotation.euler_angles());
                }
            }
            RobotCommand::LoadCommands { command_file_path } => {
                for command in load_command_file_and_filter(command_file_path.clone())? {
                    let command_parsed_iter = command.split_whitespace();
                    // Parse the command
                    let read_opt = RobotCommand::from_iter(command_parsed_iter);
                    // Execute the parsed command
                    info!("Executing {}", command);
                    self.execute(client, &read_opt).await?;
                }
            }
            RobotCommand::List => {
                println!("Raw joint trajectory clients");
                for name in client.raw_joint_trajectory_clients_names() {
                    println!(" {}", name);
                }
                println!("Joint trajectory clients");
                for name in client.joint_trajectory_clients_names() {
                    println!(" {}", name);
                }
                println!("Collision check clients");
                for name in client.collision_check_clients_names() {
                    println!(" {}", name);
                }
                println!("Ik clients");
                for name in client.ik_clients_names() {
                    println!(" {}", name);
                }
            }
            RobotCommand::Speak { message } => {
                // TODO: Parse quotations and comments
                // Currently '"Foo bar" # hoge' is parsed as message in below command.
                // 'openrr_apps_robot_command speak "Foo bar" # hoge'
                client.speak(&message.join(" "));
            }
            RobotCommand::ExecuteCommand { command } => {
                let mut iter = command.iter();
                let cmd_str = iter
                    .next()
                    .ok_or_else(|| OpenrrCommandError::NoCommand(command.to_owned()))?;
                let output = Command::new(cmd_str).args(iter).output().map_err(|e| {
                    OpenrrCommandError::CommandExecutionFailure(command.to_owned(), e)
                })?;
                if output.status.success() {
                    info!("{}", String::from_utf8_lossy(&output.stdout));
                } else {
                    error!("{}", String::from_utf8_lossy(&output.stdout));
                    error!("{}", String::from_utf8_lossy(&output.stderr));
                    return Err(OpenrrCommandError::CommandFailure(
                        command.to_owned(),
                        String::from_utf8_lossy(&output.stderr).to_string(),
                    ));
                }
            }
            RobotCommand::GetNavigationCurrentPose => {
                println!("Base Pose {}", client.current_pose()?);
            }
            RobotCommand::SendNavigationGoal {
                x,
                y,
                yaw,
                frame_id,
                timeout_secs,
            } => {
                client
                    .send_pose(
                        Isometry2::new(Vector2::new(*x, *y), *yaw),
                        frame_id,
                        Duration::from_secs_f64(*timeout_secs),
                    )
                    .await?;
            }
            RobotCommand::CancelNavigationGoal => {
                client.cancel()?;
            }
            RobotCommand::SendBaseVelocity {
                x,
                y,
                theta,
                duration_secs,
            } => {
                let start = Instant::now();
                let duration = Duration::from_secs_f64(*duration_secs);
                let sleep_duration = Duration::from_secs_f64(0.01);
                while start.elapsed() < duration {
                    client.send_velocity(&BaseVelocity::new(*x, *y, *theta))?;
                    sleep(sleep_duration);
                }
            }
        }
        Ok(())
    }
}

pub fn load_command_file_and_filter(file_path: PathBuf) -> Result<Vec<String>, OpenrrCommandError> {
    let file = File::open(&file_path)
        .map_err(|e| OpenrrCommandError::CommandFileOpenFailure(file_path, e.to_string()))?;
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
