use thiserror::Error;

#[derive(Error, Debug)]
#[non_exhaustive]
pub enum Error {
    #[error("iced: {:?}", .0)]
    Iced(#[from] iced::Error),
    #[error("urdf: {:?}", .0)]
    Urdf(#[from] urdf_rs::UrdfError),
    #[error("arci: {:?}", .0)]
    Arci(#[from] arci::Error),
}
