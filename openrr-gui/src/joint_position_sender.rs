use std::{collections::HashMap, f64, sync::Arc, time::Duration, usize};

use arci::{JointTrajectoryClient, Localization, MoveBase, Navigation};
use iced::{
    button, pick_list, scrollable, slider, text_input, window, Application, Button, Clipboard,
    Column, Command, Container, Element, HorizontalAlignment, Length, PickList, Row, Scrollable,
    Settings, Slider, Text, TextInput,
};
use openrr_client::RobotClient;
use rand::Rng;
use tracing::{debug, debug_span, error, warn};
use urdf_rs::JointType;

use crate::{style, Error};

const THEME: style::Theme = style::Theme;

/// Launches GUI that send joint positions from GUI to the given `robot_client`.
pub fn joint_position_sender<L, M, N>(
    robot_client: RobotClient<L, M, N>,
    robot: urdf_rs::Robot,
) -> Result<(), Error>
where
    L: Localization + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    let joints = joint_map(robot);
    validate_joints(&joints, &robot_client)?;

    let gui = JointPositionSender::new(robot_client, joints)?;

    // Should we expose some of the settings to the user?
    let settings = Settings {
        flags: Some(gui),
        window: window::Settings {
            size: (400, 550),
            ..window::Settings::default()
        },
        ..Settings::default()
    };

    JointPositionSender::run(settings)?;
    Ok(())
}

#[derive(Default)]
struct JointState {
    name: String,
    slider: slider::State,
    position: f64,
    position_input: String,
    position_input_state: text_input::State,
}

impl JointState {
    fn update_position(&mut self, position: f64) {
        self.position = position;
        self.position_input = format!("{position:.2}");
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

struct JointPositionSender<L, M, N>
where
    L: Localization + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    robot_client: RobotClient<L, M, N>,
    joints: HashMap<String, urdf_rs::Joint>,

    joint_trajectory_client_names: Vec<String>,
    // pick list for joint_trajectory_clients
    pick_list: pick_list::State<String>,
    current_joint_trajectory_client: String,

    scroll: scrollable::State,
    randomize_button: button::State,
    zero_button: button::State,

    // TODO: Currently, we have separate states for each joint_trajectory_client,
    // but we initialize/update joint_positions based on current_joint_positions
    // when joint_trajectory_client changed. Do we really need to separate state?
    joint_states: HashMap<String, Vec<JointState>>,

    duration: Duration,
    duration_input: String,
    duration_input_state: text_input::State,

    errors: Errors,
}

#[derive(Debug, Default)]
struct Errors {
    joint_states: Option<(usize, String)>,
    duration_input: Option<String>,
    other: Option<String>,
    update_on_error: bool,
}

impl Errors {
    fn is_none(&self) -> bool {
        self.joint_states.is_none() && self.duration_input.is_none() && self.other.is_none()
    }

    fn skip_update(&mut self, message: &Message) -> bool {
        self.update_on_error = false;
        // update always if there is no error.
        if self.is_none() {
            return false;
        }

        if self.joint_states.is_none() && self.duration_input.is_none() && self.other.is_some() {
            self.other = None;
            // Other errors are usually unresolvable by user's action.
            return false;
        }

        match message {
            Message::DurationTextInputChanged(..) if self.duration_input.is_some() => false,
            Message::SliderChanged { index, .. }
            | Message::SliderTextInputChanged { index, .. }
                if self.joint_states.is_some()
                    && self.joint_states.as_ref().unwrap().0 == *index =>
            {
                false
            }
            Message::ZeroButtonPressed | Message::RandomizeButtonPressed
                if self.joint_states.is_some() =>
            {
                false
            }
            _ => {
                self.update_on_error = true;
                true
            }
        }
    }
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
    ) -> Result<Self, Error> {
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
                            position_input: "0.00".into(),
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
            pick_list: Default::default(),
            scroll: Default::default(),
            randomize_button: Default::default(),
            zero_button: Default::default(),
            joint_states,
            duration: Duration::from_secs_f64(0.1),
            duration_input: "0.1".into(),
            duration_input_state: Default::default(),
            errors: Default::default(),
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

#[derive(Debug, Clone)]
enum Message {
    ZeroButtonPressed,
    RandomizeButtonPressed,
    SliderChanged { index: usize, position: f64 },
    SliderTextInputChanged { index: usize, position: String },
    DurationTextInputChanged(String),
    PickListChanged(String),
}

impl<L, M, N> Application for JointPositionSender<L, M, N>
where
    L: Localization + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    type Executor = iced::executor::Default;
    // Wrap Self in Option due to Self doesn't implement Default.
    type Flags = Option<Self>;
    type Message = Message;

    fn new(flags: Self::Flags) -> (Self, Command<Message>) {
        (flags.unwrap(), Command::none())
    }

    fn title(&self) -> String {
        "Joint Position Sender".into()
    }

    fn update(&mut self, message: Message, _clipboard: &mut Clipboard) -> Command<Message> {
        let debug_span = |joint_trajectory_client| {
            debug_span!("Joint Position Sender", ?joint_trajectory_client)
        };

        let span = debug_span(&self.current_joint_trajectory_client);
        let _guard = span.enter();

        if self.errors.skip_update(&message) {
            debug!("skip update");
            return Command::none();
        }

        match message {
            Message::PickListChanged(client_name) => {
                drop(_guard);
                let span = debug_span(&client_name);
                let _guard = span.enter();

                self.current_joint_trajectory_client = client_name;
                let joint_trajectory_client = self.current_joint_trajectory_client();
                let len = joint_trajectory_client.joint_names().len();

                // Initializing joint_positions based on current_joint_positions.
                let positions = match joint_trajectory_client.current_joint_positions() {
                    Ok(positions) => positions,
                    Err(e) => {
                        error!("{e}");
                        self.errors.other = Some(e.to_string());
                        vec![Default::default(); len]
                    }
                };
                for (index, position) in positions.into_iter().enumerate() {
                    self.joint_states
                        .get_mut(&self.current_joint_trajectory_client)
                        .unwrap()[index]
                        .update_position(position);
                }
                return Command::none();
            }
            Message::RandomizeButtonPressed => {
                self.errors.joint_states = None;

                for joint_state in self
                    .joint_states
                    .get_mut(&self.current_joint_trajectory_client)
                    .unwrap()
                {
                    let limit = &self.joints[&joint_state.name].limit;
                    joint_state
                        .update_position(rand::thread_rng().gen_range(limit.lower..=limit.upper));
                }
            }
            Message::ZeroButtonPressed => {
                self.errors.joint_states = None;

                for joint_state in self
                    .joint_states
                    .get_mut(&self.current_joint_trajectory_client)
                    .unwrap()
                {
                    joint_state.update_position(0.0);
                }
            }
            Message::SliderChanged {
                index,
                mut position,
            } => {
                position = round_f64(position);

                self.errors.joint_states = None;

                let joint_state = &mut self
                    .joint_states
                    .get_mut(&self.current_joint_trajectory_client)
                    .unwrap()[index];

                let limit = &self.joints[&joint_state.name].limit;

                // The position specified by the slider is guaranteed to be in range,
                // but it may actually be out of range because the value is rounded.
                // So, if the position is out of the range, handle it as the same
                // value as the limit.
                position = position.clamp(limit.lower, limit.upper);

                if (position * 100.0) as i64 == (joint_state.position * 100.0) as i64 {
                    joint_state.update_position(position);
                    // Ignore if the position has not changed at all.
                    return Command::none();
                }

                joint_state.update_position(position);
            }
            Message::SliderTextInputChanged { index, position } => {
                let joint_state = &mut self
                    .joint_states
                    .get_mut(&self.current_joint_trajectory_client)
                    .unwrap()[index];
                joint_state.position_input = position;

                match joint_state.position_input.parse::<f64>() {
                    Ok(position) => {
                        // We don't round the input for now. If we do that, we also need to update
                        // the text input that we show to the user.

                        let limit = &self.joints[&joint_state.name].limit;
                        if (limit.lower..=limit.upper).contains(&position) {
                            self.errors.joint_states = None;

                            if (position * 100.0) as i64 == (joint_state.position * 100.0) as i64 {
                                // Ignore if the position has not changed at all.
                                return Command::none();
                            }

                            joint_state.position = position;
                        } else {
                            let msg = format!(
                                "Position for joint `{}` is out of limit",
                                joint_state.name
                            );
                            warn!(?joint_state.position_input, ?limit.lower, ?limit.upper, ?msg);
                            self.errors.joint_states = Some((index, msg));
                            return Command::none();
                        }
                    }
                    Err(e) => {
                        let msg = format!(
                            "Position for joint `{}` is not a valid number",
                            joint_state.name
                        );
                        warn!(?joint_state.position_input, ?msg, "error=\"{e}\"");
                        self.errors.joint_states = Some((index, msg));
                        return Command::none();
                    }
                }
            }
            Message::DurationTextInputChanged(duration) => {
                self.duration_input = duration;

                match self.duration_input.parse::<f64>() {
                    Ok(duration) => {
                        // We don't round the input for now. If we do that, we also need to update
                        // the text input that we show to the user.

                        if !duration.is_normal() && !duration.is_sign_positive() {
                            let msg = "Duration is not a normal number".to_string();
                            warn!(?self.duration_input, ?msg);
                            self.errors.duration_input = Some(msg);
                            return Command::none();
                        }
                        if !duration.is_sign_positive() {
                            let msg = "Duration is not a positive number".to_string();
                            warn!(?self.duration_input, ?msg);
                            self.errors.duration_input = Some(msg);
                            return Command::none();
                        }

                        self.errors.duration_input = None;
                        self.duration = Duration::from_secs_f64(duration);
                        return Command::none();
                    }
                    Err(e) => {
                        let msg = "Duration is not a valid number".to_string();
                        warn!(?self.duration_input, ?msg, "error=\"{e}\"");
                        self.errors.duration_input = Some(msg);
                        return Command::none();
                    }
                }
            }
        }

        let joint_positions = self.current_joint_positions();
        let joint_trajectory_client = self.current_joint_trajectory_client();
        let duration = self.duration;
        debug!(?joint_positions, ?duration, "send_joint_positions");
        match joint_trajectory_client.send_joint_positions(joint_positions, duration) {
            Err(e) => {
                error!("{e}");
                self.errors.other = Some(e.to_string());
            }
            // do not wait
            Ok(wait) => drop(wait),
        }
        Command::none()
    }

    fn view(&mut self) -> Element<'_, Message> {
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

        let zero_button = Button::new(
            &mut self.zero_button,
            Text::new("Zero")
                .horizontal_alignment(HorizontalAlignment::Center)
                .width(Length::Fill),
        )
        .padding(10)
        .on_press(Message::ZeroButtonPressed)
        .style(THEME)
        .width(Length::Fill);

        let mut sliders = Scrollable::new(&mut self.scroll)
            .style(THEME)
            .width(Length::Fill)
            .height(Length::Fill);
        for (index, joint_state) in self
            .joint_states
            .get_mut(&self.current_joint_trajectory_client)
            .unwrap()
            .iter_mut()
            .enumerate()
        {
            let limit = &self.joints[&joint_state.name].limit;
            let slider = Slider::new(
                &mut joint_state.slider,
                limit.lower..=limit.upper,
                joint_state.position,
                move |position| Message::SliderChanged { index, position },
            )
            .style(THEME)
            .step(0.01);

            let joint_name = Text::new(&self.joints[&joint_state.name].name)
                .horizontal_alignment(HorizontalAlignment::Left)
                .width(Length::Fill);

            // TODO: set horizontal_alignment on TextInput, once https://github.com/hecrj/iced/pull/373 merged and released.
            let current_position = TextInput::new(
                &mut joint_state.position_input_state,
                "",
                &joint_state.position_input,
                move |position| Message::SliderTextInputChanged { index, position },
            )
            .style(match self.errors.joint_states {
                Some((i, _)) if i == index => style::TextInput::Error,
                _ => style::TextInput::Default,
            });

            let content = Column::new()
                .push(Row::new().push(joint_name).push(current_position))
                .push(slider);
            sliders = sliders.push(content);
        }

        let duration = Row::new()
            .spacing(10)
            .padding(5)
            .push(Text::new("Duration (sec)").width(Length::Fill))
            .push(
                TextInput::new(
                    &mut self.duration_input_state,
                    "",
                    &self.duration_input,
                    Message::DurationTextInputChanged,
                )
                .style(if self.errors.duration_input.is_none() {
                    style::TextInput::Default
                } else {
                    style::TextInput::Error
                }),
            );

        let mut content = Column::new()
            .spacing(20)
            .padding(20)
            .max_width(400)
            .height(Length::Fill);
        if let Some(pick_list) = pick_list {
            content = content.push(pick_list);
        }
        content = content
            .push(randomize_button)
            .push(zero_button)
            .push(sliders)
            .push(duration);

        if self.errors.is_none() {
            content = content.push(
                Text::new(" ")
                    .size(style::ERROR_TEXT_SIZE)
                    .width(Length::Fill),
            );
        } else {
            let mut errors = Column::new().max_width(400);
            for msg in [
                self.errors.joint_states.as_ref().map(|(_, msg)| msg),
                self.errors.duration_input.as_ref(),
                self.errors.other.as_ref(),
            ]
            .iter()
            .filter_map(|e| e.as_ref())
            {
                errors = errors.push(
                    Text::new(&format!("Error: {msg}"))
                        .size(style::ERROR_TEXT_SIZE)
                        .horizontal_alignment(HorizontalAlignment::Left)
                        .width(Length::Fill)
                        .color(style::ERRORED),
                );
            }
            if self.errors.update_on_error {
                errors = errors.push(
                    Text::new("Error: Please resolve the above error first")
                        .size(style::ERROR_TEXT_SIZE)
                        .horizontal_alignment(HorizontalAlignment::Left)
                        .width(Length::Fill)
                        .color(style::ERRORED),
                );
            }
            content = content.push(errors);
        }

        Container::new(content)
            .width(Length::Fill)
            .height(Length::Fill)
            .center_x()
            .center_y()
            .style(THEME)
            .into()
    }
}

// round float: https://stackoverflow.com/questions/28655362/how-does-one-round-a-floating-point-number-to-a-specified-number-of-digits
fn round_f64(n: f64) -> f64 {
    let n = format!("{n:.2}");
    n.parse().unwrap()
}
