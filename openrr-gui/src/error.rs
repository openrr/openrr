use thiserror::Error;

#[derive(Error, Debug)]
#[non_exhaustive]
pub enum Error {
    #[error("iced: {:?}", .0)]
    Iced(String),
    #[error("urdf: {:?}", .0)]
    Urdf(#[from] urdf_rs::UrdfError),
    #[error("arci: {:?}", .0)]
    Arci(#[from] arci::Error),
}

impl From<iced::Error> for Error {
    fn from(e: iced::Error) -> Self {
        // TODO: iced::Error doesn't implement Send/Sync, so for now, convert it to a string.
        // Without this, it cannot be used in combination with `anyhow::Result`.
        // https://github.com/hecrj/iced/issues/516
        Self::Iced(e.to_string())
    }
}
