//! [`arci::Speaker`] implementation for playing audio files.

#![warn(rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

use std::{
    collections::HashMap,
    fs::File,
    io,
    path::{Path, PathBuf},
};

use arci::{Speaker, WaitFuture};
use thiserror::Error;
use tokio::sync::oneshot;
use tracing::error;

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
            Some(path) => {
                let (sender, receiver) = oneshot::channel();
                let path = path.to_owned();

                std::thread::spawn(move || {
                    let res = play_audio_file(&path);
                    let _ = sender.send(res);
                });

                Ok(WaitFuture::new(async move {
                    receiver
                        .await
                        .map_err(|e| arci::Error::Other(e.into()))?
                        .map_err(|e| arci::Error::Other(e.into()))
                }))
            }
            None => Err(Error::HashNotFound(message.to_string())),
        }
        .map_err(|e| arci::Error::Other(e.into()))
    }
}

fn play_audio_file(path: &Path) -> Result<(), Error> {
    // NOTE: Dropping `_stream` stops the audio from playing.
    let (_stream, stream_handle) = rodio::OutputStream::try_default()?;
    let sink = rodio::Sink::try_new(&stream_handle)?;
    let file = File::open(path)?;
    let source = rodio::Decoder::new(io::BufReader::new(file))?;
    sink.append(source);
    sink.sleep_until_end();
    Ok(())
}
