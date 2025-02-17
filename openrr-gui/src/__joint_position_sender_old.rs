use std::{collections::HashMap, f64, path::Path, sync::Arc, time::Duration, usize};

use arci::{JointTrajectoryClient, Localization, MoveBase, Navigation};
use iced::{
    alignment, button, pick_list, scrollable, slider, text_input, window, Application, Column,
    Command, Element, Length, Row, Settings, Text,
};
use iced_style_config::ReloadableTheme as Theme;
use openrr_client::RobotClient;
use rand::Rng;
use tracing::{debug, debug_span, error, warn};
use urdf_rs::JointType;

use crate::{style, Error};

/// Launches GUI that send joint positions from GUI to the given `robot_client`.
pub fn joint_position_sender<L, M, N>(
    robot_client: RobotClient<L, M, N>,
    robot: urdf_rs::Robot,
    theme_path: Option<&Path>,
) -> Result<(), Error>
where
    L: Localization + 'static,
    M: MoveBase + 'static,
    N: Navigation + 'static,
{
    let joints = joint_map(robot);
    validate_joints(&joints, &robot_client)?;

    let theme = style::theme(theme_path)?;

    let gui = JointPositionSender::new(robot_client, joints, theme)?;

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
    theme: Theme,
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
        theme: Theme,
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
            theme,
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
    ReloadTheme,
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

    fn update(&mut self, message: Message) -> Command<Message> {
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
            Message::ReloadTheme => {
                if let Err(e) = self.theme.reload() {
                    error!("{e}");
                    self.errors.other = Some(e.to_string());
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
            let pick_list = self
                .theme
                .pick_list()
                .new(
                    &mut self.pick_list,
                    &self.joint_trajectory_client_names,
                    Some(self.current_joint_trajectory_client.clone()),
                    Message::PickListChanged,
                )
                .width(Length::Fill);
            Some(pick_list)
        } else {
            None
        };

        let randomize_button = self
            .theme
            .button()
            .new(
                &mut self.randomize_button,
                Text::new("Randomize")
                    .horizontal_alignment(alignment::Horizontal::Center)
                    .width(Length::Fill),
            )
            .padding(10)
            .on_press(Message::RandomizeButtonPressed)
            .width(Length::Fill);

        let zero_button = self
            .theme
            .button()
            .new(
                &mut self.zero_button,
                Text::new("Zero")
                    .horizontal_alignment(alignment::Horizontal::Center)
                    .width(Length::Fill),
            )
            .padding(10)
            .on_press(Message::ZeroButtonPressed)
            .width(Length::Fill);

        let mut sliders = self
            .theme
            .scrollable()
            .new(&mut self.scroll)
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
            let slider = self
                .theme
                .slider()
                .new(
                    &mut joint_state.slider,
                    limit.lower..=limit.upper,
                    joint_state.position,
                    move |position| Message::SliderChanged { index, position },
                )
                .step(0.01);

            let joint_name = Text::new(&self.joints[&joint_state.name].name)
                .horizontal_alignment(alignment::Horizontal::Left)
                .width(Length::Fill);

            // TODO: set horizontal_alignment on TextInput, once https://github.com/hecrj/iced/pull/373 merged and released.
            let current_position = match self.errors.joint_states {
                Some((i, _)) if i == index => &self.theme.text_input()["error"],
                _ => self.theme.text_input(),
            }
            .new(
                &mut joint_state.position_input_state,
                "",
                &joint_state.position_input,
                move |position| Message::SliderTextInputChanged { index, position },
            );

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
                if self.errors.duration_input.is_none() {
                    self.theme.text_input()
                } else {
                    &self.theme.text_input()["error"]
                }
                .new(
                    &mut self.duration_input_state,
                    "",
                    &self.duration_input,
                    Message::DurationTextInputChanged,
                ),
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
            content = content.push(self.theme.text()["error"].new(" ").width(Length::Fill));
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
                    self.theme.text()["error"]
                        .new(&format!("Error: {msg}"))
                        .width(Length::Fill),
                );
            }

            if self.errors.update_on_error {
                errors = errors.push(
                    self.theme.text()["error"]
                        .new("Error: Please resolve the above error first")
                        .width(Length::Fill),
                );
            }
            content = content.push(errors);
        }

        self.theme.container().new(content).into()
    }

    fn subscription(&self) -> iced::Subscription<Self::Message> {
        self.theme.subscription().map(|_| Message::ReloadTheme)
    }
}

// round float: https://stackoverflow.com/questions/28655362/how-does-one-round-a-floating-point-number-to-a-specified-number-of-digits
fn round_f64(n: f64) -> f64 {
    let n = format!("{n:.2}");
    n.parse().unwrap()
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
            slider: slider::State::new(),
            position: 0.,
            position_input: String::from("0.00"),
            position_input_state: text_input::State::new(),
        };
        joint_state.update_position(1.);
        assert_approx_eq!(joint_state.position, 1.);
        assert_eq!(joint_state.position_input, String::from("1.00"));
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
    fn test_error() {
        let mut errors = Errors {
            joint_states: Some((10, String::from("ten"))),
            duration_input: Some(String::from("duration")),
            other: Some(String::from("option")),
            update_on_error: false,
        };

        assert!(!errors.skip_update(&Message::DurationTextInputChanged(String::new())));
        assert!(!errors.skip_update(&Message::SliderChanged {
            index: 10,
            position: 2.
        }));
        assert!(!errors.skip_update(&Message::ZeroButtonPressed));
        assert!(errors.skip_update(&Message::PickListChanged(String::new())));
        assert!(errors.update_on_error);

        errors.joint_states = None;
        errors.duration_input = None;
        assert!(!errors.skip_update(&Message::ZeroButtonPressed));
        assert!(errors.is_none());
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

        let joint_position_sender =
            JointPositionSender::new(robot_client, dummy_joint(), style::theme(None).unwrap())
                .unwrap();

        assert_eq!(
            joint_position_sender.current_joint_trajectory_client,
            String::from("dummy")
        );
        assert_approx_eq!(joint_position_sender.duration.as_secs_f64(), 0.1);
    }

    #[test]
    fn test_application() {
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
        let joint_position_sender =
            JointPositionSender::new(robot_client, dummy_joint(), style::theme(None).unwrap())
                .unwrap();

        let (mut application, _): (
            JointPositionSender<DummyLocalization, DummyMoveBase, DummyNavigation>,
            Command<Message>,
        ) = Application::new(Some(joint_position_sender));
        assert_eq!(application.title(), String::from("Joint Position Sender"));

        application.update(Message::RandomizeButtonPressed);
        assert!(application.joint_states["dummy"][0].position.abs() > 0.0);

        application.update(Message::ZeroButtonPressed);
        assert_approx_eq!(application.joint_states["dummy"][0].position, 0.0);

        application.update(Message::SliderChanged {
            index: 0,
            position: 1.5,
        });
        assert_approx_eq!(application.joint_states["dummy"][0].position, 1.5);
        assert_eq!(
            application.joint_states["dummy"][0].position_input,
            String::from("1.50")
        );

        application.update(Message::SliderTextInputChanged {
            index: 0,
            position: String::from("2.72"),
        });
        assert_approx_eq!(application.joint_states["dummy"][0].position, 2.72);
        assert_eq!(
            application.joint_states["dummy"][0].position_input,
            String::from("2.72")
        );

        application.update(Message::SliderTextInputChanged {
            index: 0,
            position: String::from("6.28"),
        });
        assert!(application.errors.joint_states.is_some());

        application.errors.joint_states = None;
        application.update(Message::SliderTextInputChanged {
            index: 0,
            position: String::from("pi"),
        });
        assert!(application.errors.joint_states.is_some());

        application.errors.joint_states = None;
        application.update(Message::DurationTextInputChanged(String::from("-1e400")));
        assert!(application.errors.duration_input.is_some());

        application.errors.duration_input = None;
        application.update(Message::DurationTextInputChanged(String::from("-1")));
        assert!(application.errors.duration_input.is_some());

        application.update(Message::DurationTextInputChanged(String::from("0.5")));
        assert!(application.errors.duration_input.is_none());

        application.update(Message::DurationTextInputChanged(String::from("one")));
        assert!(application.errors.duration_input.is_some());
    }

    #[test]
    fn test_round_f64() {
        let num = 0.12324;
        assert_approx_eq!(round_f64(num), 0.12);
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
                        xyz: [0., 1., 2.],
                        rpy: [3., 4., 5.],
                    },
                    parent: LinkName {
                        link: String::from("dummy_parent1"),
                    },
                    child: LinkName {
                        link: String::from("dummy_child1"),
                    },
                    axis: Axis { xyz: [6., 7., 8.] },
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
                        xyz: [1., 1., 2.],
                        rpy: [3., 4., 5.],
                    },
                    parent: LinkName {
                        link: String::from("dummy_parent2"),
                    },
                    child: LinkName {
                        link: String::from("dummy_child2"),
                    },
                    axis: Axis { xyz: [6., 7., 8.] },
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
