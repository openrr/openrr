use std::{collections::HashMap, path::Path, sync::Arc, time::Duration, usize};

use arci::{JointTrajectoryClient, MoveBase, Navigation, Speaker};
#[allow(unused_imports)]
use iced::{
    button, scrollable, slider, window, Align, Application, Button, Checkbox, Column, Command,
    Container, Element, HorizontalAlignment, Length, PickList, ProgressBar, Radio, Row, Rule,
    Sandbox, Scrollable, Settings, Slider, Space, Text, TextInput,
};
use iced::{pick_list, text_input};
use log::{debug, error, warn};
use openrr_client::RobotClient;
use rand::Rng;

use crate::{style, Error};

const THEME: style::Theme = style::Theme;

#[derive(Default)]
struct JointState {
    name: String,
    slider: slider::State,
    text_input: text_input::State,
    position: f64,
}

struct Robot {
    _robot: urdf_rs::Robot,
    joints: HashMap<String, urdf_rs::Joint>,
}

impl From<urdf_rs::Robot> for Robot {
    fn from(robot: urdf_rs::Robot) -> Self {
        Self {
            joints: robot
                .joints
                .iter()
                .map(|joint| (joint.name.clone(), joint.clone()))
                .collect(),
            _robot: robot,
        }
    }
}

/// Launches GUI that send joint positions from GUI to the given `robot_client`.
pub fn joint_position_sender<S, M, N>(
    robot_client: RobotClient<S, M, N>,
    urdf_path: impl AsRef<Path>,
) -> Result<(), Error>
where
    S: Speaker + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    // TODO: If the urdf file read by `robot_client` and this urdf file are different, we should emit an error.
    let robot = urdf_rs::read_file(urdf_path)?;

    let mut joint_trajectory_client_names = robot_client.joint_trajectory_clients_names();
    joint_trajectory_client_names.sort_unstable();
    debug!("{:?}", joint_trajectory_client_names);

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

    let mut gui = JointPositionSender {
        robot_client,
        robot: robot.into(),
        current_joint_trajectory_client: joint_trajectory_client_names[0].clone(),
        joint_trajectory_client_names,
        pick_list: Default::default(),
        scroll: Default::default(),
        randomize_button: Default::default(),
        center_button: Default::default(),
        joint_states,
    };

    let joint_trajectory_client = gui.current_joint_trajectory_client();
    for (index, position) in joint_trajectory_client
        .current_joint_positions()?
        .into_iter()
        .enumerate()
    {
        gui.joint_states
            .get_mut(&gui.current_joint_trajectory_client)
            .unwrap()[index]
            .position = position;
    }

    // Should we expose some of the settings to the user?
    let settings = Settings {
        flags: Some(gui),
        window: window::Settings {
            size: (400, 500),
            ..window::Settings::default()
        },
        ..Settings::default()
    };

    JointPositionSender::run(settings)?;
    Ok(())
}

struct JointPositionSender<S, M, N>
where
    S: Speaker + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    robot_client: RobotClient<S, M, N>,
    robot: Robot,

    joint_trajectory_client_names: Vec<String>,
    // pick list for joint_trajectory_clients
    pick_list: pick_list::State<String>,
    current_joint_trajectory_client: String,

    scroll: scrollable::State,
    randomize_button: button::State,
    center_button: button::State,
    // TODO: Currently, we have separate states for each joint_trajectory_client,
    // but we initialize/update joint_positions based on current_joint_positions
    // when joint_trajectory_client changed. Do we really need to separate state?
    joint_states: HashMap<String, Vec<JointState>>,
}

impl<S, M, N> JointPositionSender<S, M, N>
where
    S: Speaker + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
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

#[derive(Debug, Clone)]
enum Message {
    Ignore,
    // Update all positions in current joint_trajectory_client
    UpdateAll(Vec<f64>),
    CenterButtonPressed,
    RandomizeButtonPressed,
    SliderChanged { index: usize, position: f64 },
    TextInputChanged { index: usize, position: String },
    PickListChanged(String),
}

impl<S, M, N> Application for JointPositionSender<S, M, N>
where
    S: Speaker + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    type Executor = iced::executor::Default;
    type Message = Message;
    // wrap in option due to Self doesn't impl Deffault.
    type Flags = Option<Self>;

    fn new(flags: Self::Flags) -> (Self, Command<Message>) {
        (flags.unwrap(), Command::none())
    }

    fn title(&self) -> String {
        "Joint Position Sender".into()
    }

    fn update(&mut self, message: Message) -> Command<Message> {
        match message {
            Message::Ignore => return Command::none(),
            Message::PickListChanged(client_name) => {
                self.current_joint_trajectory_client = client_name;
                let joint_trajectory_client = self.current_joint_trajectory_client();
                let len = joint_trajectory_client.joint_names().len();
                return Command::perform(
                    // Initializing joint_positions based on current_joint_positions.
                    async move { joint_trajectory_client.current_joint_positions() },
                    move |res| match res {
                        Ok(positions) => Message::UpdateAll(positions),
                        Err(e) => {
                            error!("{}", e);
                            // TODO: Decide how to handle errors.
                            Message::UpdateAll(vec![Default::default(); len])
                        }
                    },
                );
            }
            Message::UpdateAll(positions) => {
                for (index, position) in positions.into_iter().enumerate() {
                    self.joint_states
                        .get_mut(&self.current_joint_trajectory_client)
                        .unwrap()[index]
                        .position = position;
                }
            }
            Message::RandomizeButtonPressed => {
                for joint_state in self
                    .joint_states
                    .get_mut(&self.current_joint_trajectory_client)
                    .unwrap()
                {
                    let limit = &self.robot.joints[&joint_state.name].limit;
                    joint_state.position = rand::thread_rng().gen_range(limit.lower..=limit.upper);
                }
            }
            Message::CenterButtonPressed => {
                for joint_state in self
                    .joint_states
                    .get_mut(&self.current_joint_trajectory_client)
                    .unwrap()
                {
                    // TODO: is the center always 0.00?
                    joint_state.position = 0.0;
                }
            }
            Message::SliderChanged { index, position } => {
                let joint_state = &mut self
                    .joint_states
                    .get_mut(&self.current_joint_trajectory_client)
                    .unwrap()[index];
                if (position * 100.0) as i64 == (joint_state.position * 100.0) as i64 {
                    // Ignore if the position has not changed at all.
                    return Command::none();
                }
                joint_state.position = position;
            }
            Message::TextInputChanged { index, position } => match position.parse::<f64>() {
                Ok(position) => {
                    // round position: https://stackoverflow.com/questions/28655362/how-does-one-round-a-floating-point-number-to-a-specified-number-of-digits
                    let position = format!("{:.2}", position);
                    let position = position.parse::<f64>().unwrap();

                    let joint_state = &mut self
                        .joint_states
                        .get_mut(&self.current_joint_trajectory_client)
                        .unwrap()[index];
                    let limit = &self.robot.joints[&joint_state.name].limit;
                    if (position * 100.0) as i64 == (joint_state.position * 100.0) as i64 {
                        // Ignore if the position has not changed at all.
                        return Command::none();
                    }
                    if (limit.lower..=limit.upper).contains(&position) {
                        joint_state.position = position;
                    } else {
                        warn!(
                            "out of limit (ignored): input = {:.2}, limit {:.2}..={:.2}",
                            position, limit.lower, limit.upper
                        );
                        return Command::none();
                    }
                }
                Err(e) => {
                    warn!(
                        "invalid input (ignored): input = {}, error = {}",
                        position, e
                    );
                    return Command::none();
                }
            },
        }

        let joint_positions = self.current_joint_positions();
        let joint_trajectory_client = self.current_joint_trajectory_client();
        debug!("send_joint_positions: {:?}", joint_positions);
        Command::perform(
            async move {
                joint_trajectory_client
                    .send_joint_positions(joint_positions, Duration::from_nanos(1))
                    .await
            },
            |res| match res {
                Ok(()) => Message::Ignore,
                Err(e) => {
                    error!("{}", e);
                    // TODO: Decide how to handle errors.
                    Message::Ignore
                }
            },
        )
    }

    fn view(&mut self) -> Element<Message> {
        let pick_list = if self.joint_trajectory_client_names.len() > 1 {
            let pick_list = PickList::new(
                &mut self.pick_list,
                &self.joint_trajectory_client_names,
                Some(self.current_joint_trajectory_client.clone()),
                Message::PickListChanged,
            )
            .style(THEME)
            .width(Length::Fill);
            Some(pick_list)
        } else {
            None
        };

        let randomize_button = Button::new(
            &mut self.randomize_button,
            Text::new("Randomize")
                .horizontal_alignment(HorizontalAlignment::Center)
                .width(Length::Fill),
        )
        .padding(10)
        .on_press(Message::RandomizeButtonPressed)
        .style(THEME)
        .width(Length::Fill);

        let center_button = Button::new(
            &mut self.center_button,
            Text::new("Center")
                .horizontal_alignment(HorizontalAlignment::Center)
                .width(Length::Fill),
        )
        .padding(10)
        .on_press(Message::CenterButtonPressed)
        .style(THEME)
        .width(Length::Fill);

        let mut sliders = Scrollable::new(&mut self.scroll).style(THEME);
        for (index, joint_state) in self
            .joint_states
            .get_mut(&self.current_joint_trajectory_client)
            .unwrap()
            .iter_mut()
            .enumerate()
        {
            let limit = &self.robot.joints[&joint_state.name].limit;
            let slider = Slider::new(
                &mut joint_state.slider,
                // TODO: tests continuous joints, which range from -Pi to +Pi.
                limit.lower..=limit.upper,
                joint_state.position,
                move |position| Message::SliderChanged { index, position },
            )
            .style(THEME)
            .step(0.01);

            let joint_name = Text::new(&self.robot.joints[&joint_state.name].name)
                .horizontal_alignment(HorizontalAlignment::Left)
                .width(Length::Fill);

            // TODO: set horizontal_alignment on TextInput, once https://github.com/hecrj/iced/pull/373 merged and released.
            let current_position = TextInput::new(
                &mut joint_state.text_input,
                "",
                &format!("{:.2}", joint_state.position),
                move |position| Message::TextInputChanged { index, position },
            )
            .style(THEME);

            let content = Column::new()
                .push(Row::new().push(joint_name).push(current_position))
                .push(slider);
            sliders = sliders.push(content);
        }

        let mut content = Column::new().spacing(20).padding(20).max_width(400);
        if let Some(pick_list) = pick_list {
            content = content.push(pick_list);
        }
        content = content
            .push(randomize_button)
            .push(center_button)
            .push(sliders)
            .height(Length::Fill);

        Container::new(content)
            .width(Length::Fill)
            .height(Length::Fill)
            .center_x()
            .center_y()
            .style(THEME)
            .into()
    }
}
