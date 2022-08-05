use thiserror::Error;

#[derive(Error, Debug)]
#[non_exhaustive]
pub enum Error {
    #[error("openrr-gui: iced: {}", .0)]
    Iced(#[from] iced::Error),
    #[error("openrr-gui: iced_style_config: {}", .0)]
    IcedStyleConfig(#[from] iced_style_config::Error),
    #[error("openrr-gui: arci: {}", .0)]
    Arci(#[from] arci::Error),
    #[error("openrr-gui: other: {}", .0)]
    Other(String),
}
