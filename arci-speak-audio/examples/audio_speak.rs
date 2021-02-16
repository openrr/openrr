use arci::Speaker;
use arci_speak_audio::AudioSpeaker;
use structopt::StructOpt;
use std::collections::HashMap;

#[derive(Debug, StructOpt)]
struct Args {
    file_path: String,
}

fn main() {
    env_logger::init();
    let args = Args::from_args();
    let mut hash_map = HashMap::new();
    hash_map.insert("test".to_string(), args.file_path);
    let speaker = AudioSpeaker::new(hash_map);
    speaker.speak("test")
}
