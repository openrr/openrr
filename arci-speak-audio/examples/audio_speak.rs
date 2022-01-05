use std::{collections::HashMap, path::PathBuf};

use arci::Speaker;
use arci_speak_audio::AudioSpeaker;
use clap::Parser;

#[derive(Debug, Parser)]
struct Args {
    file_path: String,
}

#[tokio::main]
async fn main() -> Result<(), arci::Error> {
    tracing_subscriber::fmt::init();
    let args = Args::parse();
    let mut hash_map = HashMap::new();
    hash_map.insert("test".to_string(), PathBuf::from(args.file_path));
    let speaker = AudioSpeaker::new(hash_map);
    let wait = speaker.speak("test")?;
    wait.await
}
