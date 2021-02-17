use arci::Speaker;
use log::error;
use std::{collections::HashMap, fs::File, io, path::Path, path::PathBuf};

use thiserror::Error;

#[derive(Error, Debug)]
#[non_exhaustive]
pub enum Error {
    #[error("io: {:?}", .0)]
    Io(#[from] std::io::Error),
    #[error("rodio: {:?}", .0)]
    Decoder(#[from] rodio::decoder::DecoderError),
    #[error("rodio: {:?}", .0)]
    Stream(#[from] rodio::StreamError),
    #[error("rodio: {:?}", .0)]
    Play(#[from] rodio::PlayError),
    #[error("not found: {:?}", .0)]
    HashNotFound(String),
}

pub struct AudioSpeaker {
    message_to_file_path: HashMap<String, PathBuf>,
}

impl AudioSpeaker {
    /// Creates a new `AudioSpeaker`.
    pub fn new(hashmap: HashMap<String, PathBuf>) -> Self {
        Self {
            message_to_file_path: hashmap,
        }
    }

    /// Similar to `Speaker::speak`, but returns an error when the command failed.
    pub fn try_speak(&self, message: &str) -> Result<(), Error> {
        match self.message_to_file_path.get(message) {
            Some(path) => play_audio_file(path),
            None => Err(Error::HashNotFound(message.to_string())),
        }
    }
}

impl Speaker for AudioSpeaker {
    fn speak(&self, message: &str) {
        if let Err(e) = self.try_speak(message) {
            // TODO: Speaker trait seems to assume that speak method will always succeed.
            error!("{}", e);
        }
    }
}

fn play_audio_file(path: &Path) -> Result<(), Error> {
    let (_stream, stream_handle) = rodio::OutputStream::try_default()?;
    let sink = rodio::Sink::try_new(&stream_handle)?;
    let file = File::open(path)?;
    let source = rodio::Decoder::new(io::BufReader::new(file))?;
    sink.append(source);
    sink.sleep_until_end();

    Ok(())
}
