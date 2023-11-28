use std::{collections::HashMap, f64, sync::Arc, time::Duration};

use arci::{JointTrajectoryClient, Localization, MoveBase, Navigation};
use eframe::egui;
use openrr_client::RobotClient;
use rand::Rng;
use tracing::{debug, error, warn};
use urdf_rs::JointType;

use crate::Error;

/// Launches GUI that send joint positions from GUI to the given `robot_client`.
#[cfg(not(target_family = "wasm"))]
pub fn joint_position_sender<L, M, N>(
    robot_client: RobotClient<L, M, N>,
    robot: urdf_rs::Robot,
) -> Result<(), crate::Error>
where
    L: Localization + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    let joints = joint_map(robot);
    validate_joints(&joints, &robot_client)?;

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([400.0, 400.0])
            .with_icon(
                eframe::icon_data::from_png_bytes(include_bytes!("../assets/icon/openrr.png"))
                    .unwrap(),
            ),
        ..eframe::NativeOptions::default()
    };
    eframe::run_native(
        "Joint Position Sender",
        native_options,
        Box::new(|_cc| Box::new(JointPositionSender::new(robot_client, joints).unwrap())),
    )
    .map_err(|e| crate::Error::Other(e.to_string()))?; // eframe::Error is not Send
    Ok(())
}

struct JointPositionSender<L, M, N>
where
    L: Localization + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    robot_client: RobotClient<L, M, N>,
    joints: HashMap<String, urdf_rs::Joint>,

    current_joint_trajectory_client: String,
    joint_trajectory_client_names: Vec<String>,
    show_joint_trajectory_client_list: bool,

    joint_states: HashMap<String, Vec<JointState>>,

    duration: Duration,
    duration_input: String,
}

impl<L, M, N> JointPositionSender<L, M, N>
where
    L: Localization + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    fn new(
        robot_client: RobotClient<L, M, N>,
        joints: HashMap<String, urdf_rs::Joint>,
    ) -> Result<Self, crate::Error> {
        let mut joint_trajectory_client_names = robot_client.joint_trajectory_clients_names();
        joint_trajectory_client_names.sort_unstable();
        debug!("{joint_trajectory_client_names:?}");

        let joint_states = joint_trajectory_client_names
            .iter()
            .map(|client_name| {
                (
                    client_name.clone(),
                    robot_client.joint_trajectory_clients()[client_name]
                        .joint_names()
                        .iter()
                        .map(|joint_name| JointState {
                            name: joint_name.clone(),
                            ..Default::default()
                        })
                        .collect::<Vec<_>>(),
                )
            })
            .collect();

        let mut this = Self {
            robot_client,
            joints,
            current_joint_trajectory_client: joint_trajectory_client_names[0].clone(),
            joint_trajectory_client_names,
            show_joint_trajectory_client_list: false,
            joint_states,
            duration: Duration::from_secs_f64(0.1),
            duration_input: "0.1".to_owned(),
        };

        let joint_trajectory_client = this.current_joint_trajectory_client();
        for (index, position) in joint_trajectory_client
            .current_joint_positions()?
            .into_iter()
            .enumerate()
        {
            this.joint_states
                .get_mut(&this.current_joint_trajectory_client)
                .unwrap()[index]
                .update_position(position);
        }

        Ok(this)
    }

    fn current_joint_trajectory_client(&self) -> Arc<dyn JointTrajectoryClient> {
        self.robot_client.joint_trajectory_clients()[&self.current_joint_trajectory_client].clone()
    }

    fn current_joint_positions(&self) -> Vec<f64> {
        self.joint_states[&self.current_joint_trajectory_client]
            .iter()
            .map(|joint| joint.position)
            .collect()
    }
}

impl<L, M, N> eframe::App for JointPositionSender<L, M, N>
where
    L: Localization + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            egui::warn_if_debug_build(ui);

            let mut send_joint_positions = false;

            if ui.button(&self.current_joint_trajectory_client).clicked() {
                self.show_joint_trajectory_client_list ^= true;
            }
            if self.show_joint_trajectory_client_list {
                for client in &self.joint_trajectory_client_names {
                    if ui
                        .selectable_label(*client == self.current_joint_trajectory_client, client)
                        .clicked()
                    {
                        match self.robot_client.joint_trajectory_clients()[client]
                            .current_joint_positions()
                        {
                            Ok(joint_positions) => {
                                for (index, position) in joint_positions.into_iter().enumerate() {
                                    self.joint_states.get_mut(client).unwrap()[index]
                                        .update_position(position);
                                }
                                self.current_joint_trajectory_client = client.clone();
                            }
                            Err(e) => {
                                let msg = format!(
                                    "failed to get current joint positions with '{client}' client: {e:#}"
                                );
                                error!(?msg);
                                ui.colored_label(
                                    ui.visuals().error_fg_color,
                                    format!("Error: {msg:#}"),
                                );
                            }
                        }
                    }
                }
            }

            if ui.button("Randomize").clicked() {
                for joint_state in self
                    .joint_states
                    .get_mut(&self.current_joint_trajectory_client)
                    .unwrap()
                {
                    let limit = &self.joints[&joint_state.name].limit;
                    joint_state
                        .update_position(rand::thread_rng().gen_range(limit.lower..=limit.upper));
                }
                send_joint_positions = true;
            }

            if ui.button("Zero").clicked() {
                for joint_state in self
                    .joint_states
                    .get_mut(&self.current_joint_trajectory_client)
                    .unwrap()
                {
                    joint_state.update_position(0.0);
                }
                send_joint_positions = true;
            }

            for joint_state in self
                .joint_states
                .get_mut(&self.current_joint_trajectory_client)
                .unwrap()
                .iter_mut()
            {
                let limit = &self.joints[&joint_state.name].limit;
                if ui
                    .add(
                        egui::Slider::new(&mut joint_state.position, limit.lower..=limit.upper)
                            .text(&self.joints[&joint_state.name].name),
                    )
                    .changed()
                {
                    send_joint_positions = true;
                }
            }

            ui.add_space(20.0);
            ui.horizontal(|ui| {
                ui.label("Duration (sec)");
                if ui.text_edit_singleline(&mut self.duration_input).changed() {
                    match self.duration_input.parse::<f64>() {
                        Ok(duration) => {
                            // We don't round the input for now. If we do that, we also need to update
                            // the text input that we show to the user.

                            if !duration.is_normal() && !duration.is_sign_positive() {
                                let msg = "Duration is not a normal number".to_string();
                                warn!(?self.duration_input, ?msg);
                                ui.colored_label(
                                    ui.visuals().error_fg_color,
                                    format!("Error: {msg:#}"),
                                );
                            }
                            if !duration.is_sign_positive() {
                                let msg = "Duration is not a positive number".to_string();
                                warn!(?self.duration_input, ?msg);
                                ui.colored_label(
                                    ui.visuals().error_fg_color,
                                    format!("Error: {msg:#}"),
                                );
                            }

                            // self.errors.duration_input = None;
                            self.duration = Duration::from_secs_f64(duration);
                        }
                        Err(e) => {
                            let msg = "Duration is not a valid number".to_string();
                            warn!(?self.duration_input, ?msg, "error=\"{e}\"");
                            ui.colored_label(
                                ui.visuals().error_fg_color,
                                format!("Error: {msg:#}"),
                            );
                        }
                    }
                }
            });

            if send_joint_positions {
                let joint_positions = self.current_joint_positions();
                let joint_trajectory_client = self.current_joint_trajectory_client();
                let duration = self.duration;
                debug!(?joint_positions, ?duration, "send_joint_positions");
                match joint_trajectory_client.send_joint_positions(joint_positions, duration) {
                    Err(e) => {
                        error!("{e}");
                        ui.colored_label(ui.visuals().error_fg_color, format!("Error: {e:#}"));
                    }
                    // do not wait
                    Ok(wait) => drop(wait),
                }
            }
        });
    }
}

fn joint_map(mut robot: urdf_rs::Robot) -> HashMap<String, urdf_rs::Joint> {
    for joint in &mut robot.joints {
        // If limit is not specified, urdf-rs assigns f64::default.
        if JointType::Continuous == joint.joint_type {
            joint.limit.lower = -f64::consts::PI;
            joint.limit.upper = f64::consts::PI;
        }
    }

    robot
        .joints
        .into_iter()
        .map(|joint| (joint.name.clone(), joint))
        .collect()
}

/// Returns an error if the joint names returned by joint trajectory clients do not exist in `joints`.
fn validate_joints<L, M, N>(
    joints: &HashMap<String, urdf_rs::Joint>,
    client: &RobotClient<L, M, N>,
) -> Result<(), Error>
where
    L: Localization + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    for client in client.joint_trajectory_clients().values() {
        for joint_name in client.joint_names() {
            if !joints.contains_key(&joint_name) {
                return Err(Error::Other(format!(
                    "Joint '{joint_name}' not found in URDF"
                )));
            }
        }
    }
    Ok(())
}

#[derive(Default)]
struct JointState {
    name: String,
    position: f64,
    // position_input: String,
}

impl JointState {
    fn update_position(&mut self, position: f64) {
        self.position = position;
        // self.position_input = format!("{position:.2}");
    }
}

#[cfg(test)]
mod test {
    use arci::{
        DummyJointTrajectoryClient, DummyLocalization, DummyMoveBase, DummyNavigation,
        DummySpeaker, Speaker,
    };
    use assert_approx_eq::assert_approx_eq;
    use openrr_client::OpenrrClientsConfig;
    use urdf_rs::{Axis, Joint, JointLimit, LinkName, Pose, Robot};

    use super::*;

    #[test]
    fn test_joint_state() {
        let mut joint_state = JointState {
            name: String::from("joint_state"),
            position: 0.,
        };
        joint_state.update_position(1.);
        assert_approx_eq!(joint_state.position, 1.);
    }

    #[test]
    fn test_joint_map() {
        let joint = dummy_joint();

        assert_eq!(joint["dummy_joint1"].name, String::from("dummy_joint1"));
        assert_approx_eq!(joint["dummy_joint1"].limit.lower, -f64::consts::PI);
        assert_approx_eq!(joint["dummy_joint1"].limit.upper, f64::consts::PI);
        assert_approx_eq!(joint["dummy_joint2"].limit.upper, 2.7);
        assert_approx_eq!(joint["dummy_joint1"].limit.effort, 0.2);
        assert_eq!(
            joint["dummy_joint1"].child.link,
            String::from("dummy_child1")
        );
        assert_eq!(
            joint["dummy_joint2"].parent.link,
            String::from("dummy_parent2")
        );
    }

    #[test]
    fn test_validate_joints() {
        let joints = dummy_joint();

        let mut raw_joint_trajectory_clients = HashMap::from([(
            String::from("dummy"),
            Arc::new(DummyJointTrajectoryClient::new(vec![String::from(
                "dummy_joint1",
            )])) as Arc<dyn JointTrajectoryClient>,
        )]);
        let speakers = HashMap::from([(
            String::from("speaker"),
            Arc::new(DummySpeaker::new()) as Arc<dyn Speaker>,
        )]);

        let ok_client = dummy_robot_client(raw_joint_trajectory_clients.clone(), speakers.clone());

        raw_joint_trajectory_clients
            .entry(String::from("err"))
            .or_insert(Arc::new(DummyJointTrajectoryClient::new(vec![String::from(
                "no_exist_joint",
            )])) as Arc<dyn JointTrajectoryClient>);

        let err_client = dummy_robot_client(raw_joint_trajectory_clients, speakers);

        assert!(validate_joints(&joints, &ok_client).is_ok());
        assert!(validate_joints(&joints, &err_client).is_err());
    }

    #[test]
    fn test_joint_position_sender() {
        let raw_joint_trajectory_clients = HashMap::from([(
            String::from("dummy"),
            Arc::new(DummyJointTrajectoryClient::new(vec![String::from(
                "dummy_joint1",
            )])) as Arc<dyn JointTrajectoryClient>,
        )]);
        let speakers = HashMap::from([(
            String::from("speaker"),
            Arc::new(DummySpeaker::new()) as Arc<dyn Speaker>,
        )]);

        let robot_client = dummy_robot_client(raw_joint_trajectory_clients, speakers);

        let joint_position_sender = JointPositionSender::new(robot_client, dummy_joint()).unwrap();

        assert_eq!(
            joint_position_sender.current_joint_trajectory_client,
            String::from("dummy")
        );
        assert_approx_eq!(joint_position_sender.duration.as_secs_f64(), 0.1);
    }

    fn dummy_robot_client(
        raw_joint_trajectory_clients: HashMap<String, Arc<dyn JointTrajectoryClient>>,
        speakers: HashMap<String, Arc<dyn Speaker>>,
    ) -> RobotClient<DummyLocalization, DummyMoveBase, DummyNavigation> {
        RobotClient::new(
            OpenrrClientsConfig::default(),
            raw_joint_trajectory_clients,
            speakers,
            Some(DummyLocalization::new()),
            Some(DummyMoveBase::new()),
            Some(DummyNavigation::new()),
        )
        .unwrap()
    }

    fn dummy_joint() -> HashMap<String, Joint> {
        let robot = Robot {
            name: String::from("dummy"),
            links: vec![],
            joints: vec![
                Joint {
                    name: String::from("dummy_joint1"),
                    joint_type: JointType::Continuous,
                    origin: Pose {
                        xyz: urdf_rs::Vec3([0., 1., 2.]),
                        rpy: urdf_rs::Vec3([3., 4., 5.]),
                    },
                    parent: LinkName {
                        link: String::from("dummy_parent1"),
                    },
                    child: LinkName {
                        link: String::from("dummy_child1"),
                    },
                    axis: Axis {
                        xyz: urdf_rs::Vec3([6., 7., 8.]),
                    },
                    limit: JointLimit {
                        lower: 0.1,
                        upper: 2.7,
                        effort: 0.2,
                        velocity: 0.3,
                    },
                    dynamics: None,
                    mimic: None,
                    safety_controller: None,
                },
                Joint {
                    name: String::from("dummy_joint2"),
                    joint_type: JointType::Revolute,
                    origin: Pose {
                        xyz: urdf_rs::Vec3([1., 1., 2.]),
                        rpy: urdf_rs::Vec3([3., 4., 5.]),
                    },
                    parent: LinkName {
                        link: String::from("dummy_parent2"),
                    },
                    child: LinkName {
                        link: String::from("dummy_child2"),
                    },
                    axis: Axis {
                        xyz: urdf_rs::Vec3([6., 7., 8.]),
                    },
                    limit: JointLimit {
                        lower: 0.1,
                        upper: 2.7,
                        effort: 0.2,
                        velocity: 0.3,
                    },
                    dynamics: None,
                    mimic: None,
                    safety_controller: None,
                },
            ],
            materials: vec![],
        };
        joint_map(robot)
    }
}
