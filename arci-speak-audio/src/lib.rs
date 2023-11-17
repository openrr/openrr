#![doc = include_str!("../README.md")]

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

#[derive(Debug)]
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
    let file = File::open(path)?;
    let source = rodio::Decoder::new(io::BufReader::new(file))?;

    let (sender, receiver) = oneshot::channel();
    std::thread::spawn(move || {
        let res: Result<_, Error> = (|| {
            // NOTE: Dropping `_stream` stops the audio from playing.
            let (_stream, stream_handle) = rodio::OutputStream::try_default()?;
            let sink = rodio::Sink::try_new(&stream_handle)?;
            sink.append(source);
            sink.sleep_until_end();
            Ok(())
        })();
        let _ = sender.send(res);
    });

    Ok(WaitFuture::new(async move {
        receiver
            .await
            .map_err(|e| arci::Error::Other(e.into()))?
            .map_err(|e| arci::Error::Other(e.into()))
    }))
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_audio_speaker_new() {
        let audio_speaker = AudioSpeaker::new(HashMap::from([(
            String::from("name"),
            PathBuf::from("path"),
        )]));
        assert_eq!(
            audio_speaker.message_to_file_path["name"],
            PathBuf::from("path")
        );
    }

    #[test]
    fn test_audio_speaker_speak() {
        let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        let root_dir = manifest_dir.parent().unwrap();
        let audio_path = root_dir.join("openrr-apps/audio/sine.mp3");
        let audio_speaker = AudioSpeaker::new(HashMap::from([(String::from("name"), audio_path)]));

        assert!(audio_speaker.speak("name").is_ok());
        assert!(audio_speaker.speak("not_exist").is_err());
    }

    #[test]
    fn test_play_audio_file() {
        let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        let root_dir = manifest_dir.parent().unwrap();
        let audio_path = root_dir.join("openrr-apps/audio/sine.mp3");
        let fake_path = root_dir.join("fake/audio/sine.mp3");

        assert!(play_audio_file(&audio_path).is_ok());
        assert!(play_audio_file(&fake_path).is_err());
    }
}
