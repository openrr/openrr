//! [`arci::Speaker`] implementation for playing audio files.

use arci::{Speaker, WaitFuture};
use std::{collections::HashMap, fs::File, io, path::Path, path::PathBuf};
use tracing::error;

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
}

impl Speaker for AudioSpeaker {
    fn speak(&self, message: &str) -> Result<WaitFuture, arci::Error> {
        match self.message_to_file_path.get(message) {
            Some(path) => play_audio_file(path),
            None => Err(Error::HashNotFound(message.to_string())),
        }
        .map_err(|e| arci::Error::Other(e.into()))
    }
}

fn play_audio_file(path: &Path) -> Result<WaitFuture, Error> {
    let (_stream, stream_handle) = rodio::OutputStream::try_default()?;
    let sink = rodio::Sink::try_new(&stream_handle)?;
    let file = File::open(path)?;
    let source = rodio::Decoder::new(io::BufReader::new(file))?;
    sink.append(source);

    // Creates a WaitFuture that waits until the sound ends only if the future
    // is polled. This future is a bit tricky, but it's more efficient than
    // using only `tokio::task::spawn_blocking` because it doesn't spawn threads
    // if the WaitFuture is ignored.
    let wait = WaitFuture::new(async move {
        tokio::task::spawn_blocking(move || {
            sink.sleep_until_end();
        })
        .await
        .map_err(|e| arci::Error::Other(e.into()))
    });

    Ok(wait)
}
