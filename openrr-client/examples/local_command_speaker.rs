use arci::Speaker;
use core::panic;
use openrr_client::LocalCommandSpeaker;
use regex::Regex;
use std::process::Command;
use structopt::StructOpt;

#[derive(Debug, StructOpt)]
struct Args {
    message: String,
    #[structopt(long)]
    test_all: bool,
}

fn main() {
    env_logger::init();
    let args = Args::from_args();

    if args.test_all {
        if cfg!(not(target_os = "macos")) {
            eprintln!("--test-all flag currently only work on macOS");
            return;
        }

        let mut speaker = LocalCommandSpeaker::default();
        let output = Command::new("say").args(&["-v", "?"]).output().unwrap();
        assert!(output.status.success());
        let output = String::from_utf8(output.stdout).unwrap();
        let re = Regex::new(r"^\w+ \s*(?P<locale>\w+)  *# (?P<message>.*)").unwrap();
        for line in output.lines() {
            if let Some(caps) = re.captures(line) {
                let locale = caps.name("locale").unwrap().as_str();
                let message = caps.name("message").unwrap().as_str();
                speaker.locale(locale).unwrap().speak(message);
            }
        }
    } else {
        let speaker = LocalCommandSpeaker::default();
        speaker.speak(&args.message)
    }
}
