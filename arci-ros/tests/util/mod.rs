use std::{
    env,
    process::{Command, Output},
    str::from_utf8,
    thread::sleep,
    time::Duration,
};

pub use child_process_terminator::ChildProcessTerminator;

mod child_process_terminator;

fn rostopic_listing_succeeds() -> bool {
    Command::new("rostopic")
        .arg("list")
        .output()
        .unwrap()
        .status
        .success()
}

fn await_roscore() {
    while !rostopic_listing_succeeds() {
        sleep(Duration::from_millis(100));
    }
}

pub fn run_roscore(port: u32) -> ChildProcessTerminator {
    println!("Running roscore on port: {}", port);
    env::set_var("ROS_MASTER_URI", format!("http://localhost:{}", port));
    while !portpicker::is_free(port as u16) {
        println!("Waiting port={}", port);
        sleep(Duration::from_millis(100));
    }
    let roscore = ChildProcessTerminator::spawn(
        &mut Command::new("roscore").arg("-p").arg(format!("{}", port)),
    );
    await_roscore();
    roscore
}

#[allow(dead_code)]
pub fn run_roscore_for(language: Language, feature: Feature) -> ChildProcessTerminator {
    run_roscore(generate_port(language, feature))
}

#[allow(dead_code)]
pub enum Language {
    None,
    Cpp,
    Python,
    Rust,
    Shell,
    Multi,
}

impl Language {
    #[allow(dead_code)]
    fn get_offset(&self) -> u32 {
        match self {
            Language::None => 1,
            Language::Cpp => 2,
            Language::Python => 3,
            Language::Rust => 4,
            Language::Shell => 5,
            Language::Multi => 6,
        }
    }
}

#[allow(dead_code)]
pub enum Feature {
    Client,
    Service,
    Publisher,
    Subscriber,
    Log,
    Parameters,
    Benchmarks,
}

impl Feature {
    #[allow(dead_code)]
    fn get_offset(&self) -> u32 {
        match self {
            Feature::Client => 100,
            Feature::Service => 200,
            Feature::Publisher => 300,
            Feature::Subscriber => 400,
            Feature::Log => 500,
            Feature::Parameters => 600,
            Feature::Benchmarks => 700,
        }
    }
}

#[allow(dead_code)]
fn generate_port(language: Language, feature: Feature) -> u32 {
    11400 + language.get_offset() + feature.get_offset()
}

pub fn bytes_contain(sequence: &[u8], subsequence: &[u8]) -> bool {
    sequence
        .windows(subsequence.len())
        .any(|window| window == subsequence)
}

#[allow(dead_code)]
pub fn assert_success_and_output_containing(output: Output, expected: &str) {
    assert!(
        output.status.success(),
        "STDERR: {}",
        from_utf8(&output.stderr).unwrap_or("not valid UTF-8"),
    );
    let stdout = output.stdout;
    assert!(
        bytes_contain(&stdout, expected.as_bytes()),
        "expected: {}, STDOUT: {}",
        expected,
        from_utf8(&stdout).unwrap_or("not valid UTF-8")
    );
}
