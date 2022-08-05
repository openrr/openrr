use std::{f64, mem, num::FpCategory, ops::RangeInclusive, path::Path, usize};

use arci::{BaseVelocity, MoveBase};
use iced::{
    alignment, button, slider, text_input, window, Alignment, Application, Button, Column, Command,
    Element, Length, Row, Settings, Text,
};
use iced_style_config::ReloadableTheme as Theme;
use tracing::{debug, error, warn};

use crate::{style, Error};

/// Launches GUI that send base velocity from GUI to the given `move_base`.
pub fn velocity_sender<M>(move_base: M, theme_path: Option<&Path>) -> Result<(), Error>
where
    M: MoveBase + 'static,
{
    let theme = style::theme(theme_path)?;

    let gui = VelocitySender::new(move_base, theme);

    // Should we expose some of the settings to the user?
    let settings = Settings {
        flags: Some(gui),
        window: window::Settings {
            size: (400, 500),
            ..window::Settings::default()
        },
        ..Settings::default()
    };

    VelocitySender::run(settings)?;
    Ok(())
}

struct SliderState {
    name: &'static str,
    slider: slider::State,
    // sign of current value: 1, -1, or 0
    sign: i8,
    // current value
    value: f64,
    // previous value
    prev: f64,
    input: String,
    input_state: text_input::State,
    // limit
    // TODO: make limit configurable?
    range: RangeInclusive<f64>,
}

const DEFAULT_ACTIVE_VELOCITY_VALUE: f64 = 0.25;

impl SliderState {
    fn new(name: &'static str, range: RangeInclusive<f64>) -> Self {
        Self {
            name,
            slider: Default::default(),
            sign: 0,
            value: 0.0,
            prev: 0.0,
            input: "0.00".to_string(),
            input_state: Default::default(),
            range,
        }
    }

    fn is_zero(&self) -> bool {
        self.sign == 0
    }

    fn update_value(&mut self, value: f64) {
        if value.classify() == FpCategory::Zero {
            if self.sign == 0 {
                return;
            }
            self.sign = 0;
        } else {
            self.sign = value.signum() as i8;
        }
        self.prev = mem::replace(&mut self.value, value);
        self.input = format!("{:.2}", self.value);
    }

    fn restore_prev(&mut self, sign: i8) {
        self.sign = sign;
        if self.prev.classify() == FpCategory::Zero {
            self.value = DEFAULT_ACTIVE_VELOCITY_VALUE * sign as f64
        } else {
            self.value = self.prev.abs() * sign as f64;
        }
        self.input = format!("{:.2}", self.value);
    }
}

struct ButtonState {
    button: button::State,
    msg: fn() -> Message,
    // glow backend doesn't support image.
    // https://github.com/hecrj/iced/issues/846
    #[cfg(not(feature = "glow"))]
    img: iced::image::Handle,
    #[cfg(feature = "glow")]
    alt: &'static str,
}

macro_rules! button_state {
    ($msg:expr, $img_path:expr, $alt:expr $(,)?) => {
        ButtonState {
            button: Default::default(),
            msg: || $msg,
            #[cfg(not(feature = "glow"))]
            img: iced::image::Handle::from_memory(
                include_bytes!(concat!("../assets/material-design-icons/", $img_path)).to_vec(),
            ),
            #[cfg(feature = "glow")]
            alt: $alt,
        }
    };
}

impl ButtonState {
    fn button(&mut self, velocity_state: &[SliderState]) -> Button<'_, Message> {
        let msg = (self.msg)();
        let selected = msg.state_sign() == velocity_state[msg.state_index()].sign;
        #[cfg(not(feature = "glow"))]
        {
            Button::new(&mut self.button, iced::Image::new(self.img.clone()))
                .padding(10)
                .style(style::Button::Round { selected })
                .on_press(msg)
        }
        #[cfg(feature = "glow")]
        {
            Button::new(
                &mut self.button,
                Text::new(self.alt)
                    .horizontal_alignment(alignment::Horizontal::Center)
                    .width(Length::Fill),
            )
            .padding(10)
            .style(style::Button::Round { selected })
            .on_press(msg)
        }
    }
}

#[derive(Debug, Default)]
struct Errors {
    velocity_state: Option<(usize, String)>,
    other: Option<String>,
    update_on_error: bool,
}

impl Errors {
    fn is_none(&self) -> bool {
        self.velocity_state.is_none() && self.other.is_none()
    }

    fn skip_update(&mut self, message: &Message) -> bool {
        self.update_on_error = false;
        // update always if there is no error.
        if self.is_none() {
            return false;
        }

        if self.velocity_state.is_none() && self.other.is_some() {
            self.other = None;
            // Other errors are usually unresolvable by user's action.
            return false;
        }

        match message {
            Message::SliderChanged { index, .. }
            | Message::SliderTextInputChanged { index, .. }
                if self.velocity_state.is_some()
                    && self.velocity_state.as_ref().unwrap().0 == *index =>
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

struct VelocitySender<M>
where
    M: MoveBase + 'static,
{
    move_base: M,

    /// x, y, theta
    velocity_state: [SliderState; 3],
    show_velocity: bool,
    /// x+
    up_button: ButtonState,
    /// x-
    down_button: ButtonState,
    /// y+
    left_button: ButtonState,
    /// y-
    right_button: ButtonState,
    /// theta+
    counterclockwise_button: ButtonState,
    /// theta-
    clockwise_button: ButtonState,
    /// set velocity to 0.
    stop_button: button::State,

    errors: Errors,
    theme: Theme,
}

impl<M> VelocitySender<M>
where
    M: MoveBase + 'static,
{
    fn new(move_base: M, theme: Theme) -> Self {
        // https://github.com/hecrj/iced/issues/846
        #[cfg(feature = "glow")]
        warn!("glow backend is not fully supported");

        Self {
            move_base,
            velocity_state: [
                SliderState::new("x", -1.0..=1.0),
                SliderState::new("y", -1.0..=1.0),
                SliderState::new("θ", -1.0..=1.0),
            ],
            show_velocity: true,
            up_button: button_state!(
                Message::UpButtonPressed,
                "baseline_arrow_upward_white_24dp.png",
                "↑",
            ),
            down_button: button_state!(
                Message::DownButtonPressed,
                "baseline_arrow_downward_white_24dp.png",
                "↓",
            ),
            left_button: button_state!(
                Message::LeftButtonPressed,
                "baseline_arrow_back_white_24dp.png",
                "←",
            ),
            right_button: button_state!(
                Message::RightButtonPressed,
                "baseline_arrow_forward_white_24dp.png",
                "→",
            ),
            counterclockwise_button: button_state!(
                Message::CounterclockwiseButtonPressed,
                "baseline_undo_white_24dp_rotate90.png",
                // ↺
                "CCW",
            ),
            clockwise_button: button_state!(
                Message::ClockwiseButtonPressed,
                "baseline_redo_white_24dp_rotate270.png",
                // ↻
                "CW",
            ),
            stop_button: Default::default(),
            errors: Default::default(),
            theme,
        }
    }

    fn current_velocity(&self) -> BaseVelocity {
        BaseVelocity {
            x: self.velocity_state[0].value,
            y: self.velocity_state[1].value,
            theta: self.velocity_state[2].value,
        }
    }
}

#[derive(Debug, Clone)]
enum Message {
    UpButtonPressed,
    DownButtonPressed,
    LeftButtonPressed,
    RightButtonPressed,
    CounterclockwiseButtonPressed,
    ClockwiseButtonPressed,
    StopButtonPressed,
    SliderChanged { index: usize, value: f64 },
    SliderTextInputChanged { index: usize, value: String },
    CheckboxToggled(bool),
    Retained,
    ReloadTheme,
}

impl Message {
    #[track_caller]
    fn state_index(&self) -> usize {
        match self {
            Self::UpButtonPressed | Self::DownButtonPressed => 0,
            Self::LeftButtonPressed | Self::RightButtonPressed => 1,
            Self::CounterclockwiseButtonPressed | Self::ClockwiseButtonPressed => 2,
            _ => unreachable!(),
        }
    }

    #[track_caller]
    fn state_sign(&self) -> i8 {
        match self {
            Self::UpButtonPressed
            | Self::LeftButtonPressed
            | Self::CounterclockwiseButtonPressed => 1,
            Self::DownButtonPressed | Self::RightButtonPressed | Self::ClockwiseButtonPressed => -1,
            _ => unreachable!(),
        }
    }
}

impl<M> Application for VelocitySender<M>
where
    M: MoveBase + 'static,
{
    type Executor = iced::executor::Default;
    // Wrap Self in Option due to Self doesn't implement Default.
    type Flags = Option<Self>;
    type Message = Message;

    fn new(flags: Self::Flags) -> (Self, Command<Message>) {
        (flags.unwrap(), Command::none())
    }

    fn title(&self) -> String {
        "Velocity Sender".into()
    }

    fn update(&mut self, message: Message) -> Command<Message> {
        if self.errors.skip_update(&message) {
            debug!("skip update");
            return Command::none();
        }

        match message {
            Message::SliderChanged { index, mut value } => {
                value = round_f64(value);

                self.errors.velocity_state = None;

                let state = &mut self.velocity_state[index];

                if (value * 100.0) as i64 == (state.value * 100.0) as i64 {
                    state.update_value(value);
                    // Ignore if the value has not changed at all.
                    return Command::none();
                }

                state.update_value(value);
            }
            Message::SliderTextInputChanged { index, value } => {
                let state = &mut self.velocity_state[index];
                state.input = value;

                match state.input.parse::<f64>() {
                    Ok(value) => {
                        // We don't round the input for now. If we do that, we also need to update
                        // the text input that we show to the user.

                        if state.range.contains(&value) {
                            self.errors.velocity_state = None;

                            if (value * 100.0) as i64 == (state.value * 100.0) as i64 {
                                // Ignore if the value has not changed at all.
                                return Command::none();
                            }

                            state.value = value;
                        } else {
                            let msg = format!("Value for `{}` is out of limit", state.name);
                            warn!(?state.input, ?state.range, ?msg);
                            self.errors.velocity_state = Some((index, msg));
                            return Command::none();
                        }
                    }
                    Err(e) => {
                        let msg = format!("Value for `{}` is not a valid number", state.name);
                        warn!(?state.input, ?msg, "error=\"{e}\"");
                        self.errors.velocity_state = Some((index, msg));
                        return Command::none();
                    }
                }
            }
            Message::CheckboxToggled(is_checked) => {
                self.show_velocity = is_checked;
                return Command::none();
            }
            Message::StopButtonPressed => {
                for state in &mut self.velocity_state {
                    state.update_value(0.0);
                }
            }
            Message::UpButtonPressed
            | Message::DownButtonPressed
            | Message::LeftButtonPressed
            | Message::RightButtonPressed
            | Message::CounterclockwiseButtonPressed
            | Message::ClockwiseButtonPressed => {
                let index = message.state_index();
                let state = &mut self.velocity_state[index];
                let mut sign = message.state_sign();
                if sign == state.sign {
                    sign = 0;
                }
                if state.is_zero() {
                    state.restore_prev(sign);
                } else {
                    state.update_value(state.value.abs() * sign as f64);
                }
            }
            Message::Retained => {}
            Message::ReloadTheme => {
                if let Err(e) = self.theme.reload() {
                    error!("{e}");
                    self.errors.other = Some(e.to_string());
                }
            }
        }

        let velocity = self.current_velocity();
        debug!(?velocity, "send_velocity");
        if let Err(e) = self.move_base.send_velocity(&velocity) {
            error!("{e}");
            self.errors.other = Some(e.to_string());
        }
        Command::none()
    }

    fn subscription(&self) -> iced::Subscription<Self::Message> {
        iced::Subscription::batch([
            iced_futures::backend::native::tokio::time::every(std::time::Duration::from_millis(
                100,
            ))
            .map(|_| Message::Retained),
            self.theme.subscription().map(|_| Message::ReloadTheme),
        ])
    }

    fn view(&mut self) -> Element<'_, Message> {
        let mut content = Column::new()
            .spacing(10)
            .padding(20)
            .max_width(400)
            .height(Length::Fill)
            .push(
                Column::new()
                    .align_items(Alignment::Center)
                    .width(Length::Fill)
                    .height(Length::Fill)
                    .push(self.up_button.button(&self.velocity_state))
                    .push(
                        Row::new()
                            .padding(10)
                            .spacing(10)
                            .push(self.clockwise_button.button(&self.velocity_state))
                            .push(self.left_button.button(&self.velocity_state))
                            .push(
                                Button::new(
                                    &mut self.stop_button,
                                    Text::new("Stop")
                                        .horizontal_alignment(alignment::Horizontal::Center)
                                        .width(Length::Fill),
                                )
                                .padding(10)
                                .style(style::Button::Round { selected: false })
                                .on_press(Message::StopButtonPressed),
                            )
                            .push(self.right_button.button(&self.velocity_state))
                            .push(self.counterclockwise_button.button(&self.velocity_state)),
                    )
                    .push(self.down_button.button(&self.velocity_state)),
            );

        if self.show_velocity {
            for (index, state) in self.velocity_state.iter_mut().enumerate() {
                let slider = self
                    .theme
                    .slider()
                    .new(
                        &mut state.slider,
                        state.range.clone(),
                        state.value,
                        move |value| Message::SliderChanged { index, value },
                    )
                    .step(0.01);

                let joint_name = Text::new(state.name)
                    .horizontal_alignment(alignment::Horizontal::Left)
                    .width(Length::Fill);

                // TODO: set horizontal_alignment on TextInput, once https://github.com/hecrj/iced/pull/373 merged and released.
                let current_value = match self.errors.velocity_state {
                    Some((i, _)) if i == index => &self.theme.text_input()["error"],
                    _ => self.theme.text_input(),
                }
                .new(&mut state.input_state, "", &state.input, move |value| {
                    Message::SliderTextInputChanged { index, value }
                });

                let c = Column::new()
                    .push(Row::new().push(joint_name).push(current_value))
                    .push(slider);
                content = content.push(c);
            }
        }

        content = content.push(
            Row::new()
                .width(Length::Fill)
                .align_items(Alignment::End)
                .push(Text::new(" ").width(Length::Fill))
                .push(
                    self.theme
                        .checkbox()
                        .new(self.show_velocity, "Show details", Message::CheckboxToggled)
                        .size(16)
                        .spacing(10),
                ),
        );

        if self.errors.is_none() {
            content = content.push(self.theme.text()["error"].new(" "));
        } else {
            let mut errors = Column::new().max_width(400);
            for msg in [
                self.errors.velocity_state.as_ref().map(|(_, msg)| msg),
                self.errors.other.as_ref(),
            ]
            .iter()
            .filter_map(|e| e.as_ref())
            {
                errors = errors.push(self.theme.text()["error"].new(&format!("Error: {msg}")));
            }
            if self.errors.update_on_error {
                errors = errors.push(
                    self.theme.text()["error"].new("Error: Please resolve the above error first"),
                );
            }
            content = content.push(errors);
        }

        self.theme.container().new(content).into()
    }
}

// round float: https://stackoverflow.com/questions/28655362/how-does-one-round-a-floating-point-number-to-a-specified-number-of-digits
fn round_f64(n: f64) -> f64 {
    let n = format!("{n:.2}");
    n.parse().unwrap()
}
