use std::path::Path;

use iced::{button, Color};
use iced_style_config::ReloadableTheme;

pub(crate) fn theme(
    #[allow(unused_variables)] path: Option<&Path>,
) -> iced_style_config::Result<ReloadableTheme> {
    #[cfg(not(target_family = "wasm"))]
    if let Some(path) = path {
        return ReloadableTheme::from_file(path);
    }
    include_str!("../dark_theme.toml").parse()
}

const ACCENT: Color = Color::from_rgb(
    0x6F as f32 / 255.0,
    0xFF as f32 / 255.0,
    0xE9 as f32 / 255.0,
);

const ACTIVE: Color = Color::from_rgb(
    0x72 as f32 / 255.0,
    0x89 as f32 / 255.0,
    0xDA as f32 / 255.0,
);

const HOVERED: Color = Color::from_rgb(
    0x67 as f32 / 255.0,
    0x7B as f32 / 255.0,
    0xC4 as f32 / 255.0,
);

pub enum Button {
    Round { selected: bool },
}

impl button::StyleSheet for Button {
    fn active(&self) -> button::Style {
        match self {
            Self::Round { selected: false } => button::Style {
                background: ACTIVE.into(),
                border_radius: 50.0,
                text_color: Color::WHITE,
                ..button::Style::default()
            },
            Self::Round { selected: true } => button::Style {
                background: ACTIVE.into(),
                border_radius: 50.0,
                border_width: 2.0,
                border_color: ACCENT,
                text_color: Color::WHITE,
                ..button::Style::default()
            },
        }
    }

    fn hovered(&self) -> button::Style {
        match self {
            Self::Round { .. } => button::Style {
                background: HOVERED.into(),
                text_color: Color::WHITE,
                ..self.active()
            },
        }
    }

    fn pressed(&self) -> button::Style {
        match self {
            Self::Round { .. } => button::Style {
                border_width: 1.0,
                border_color: Color::WHITE,
                ..self.hovered()
            },
        }
    }
}
