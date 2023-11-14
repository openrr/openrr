use std::{
    env,
    process::{Command, Output},
    str::from_utf8,
    sync::{Arc, Once, RwLock, Weak},
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
    println!("Running roscore on port: {port}");
    env::set_var("ROS_MASTER_URI", format!("http://localhost:{port}"));
    while !portpicker::is_free(port as u16) {
        println!("Waiting port={port}");
        sleep(Duration::from_millis(100));
    }
    let roscore =
        ChildProcessTerminator::spawn(Command::new("roscore").arg("-p").arg(format!("{port}")));
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

/// # initialize roscore, rosrust
///
/// ``roscore`` and rosrust is running only one.
/// This function enable to run test using roscore.
/// To strict call once its parts.
///
/// # Example
///
/// Need to be bound to a variable to maintain running ``roscore``.
/// Thus, return should not drop.
///
/// ```
/// let rosrust_init_name = String::from("ros_rust");
///
/// // Important action
/// let _roscore = run_roscore_and_rosrust_init_once(&rosrust_init_name);
///
/// ```
///
/* think that ``OnceCell`` 's method get ``panic!`` in rare cases.
 * Therefore using ``unwrap`` for error handling.
 */
pub fn run_roscore_and_rosrust_init_once(init_name: &str) -> Arc<ChildProcessTerminator> {
    use once_cell::sync::{Lazy, OnceCell};

    static ONCE: Once = Once::new();
    static PORT: Lazy<u32> = Lazy::new(|| {
        portpicker::pick_unused_port()
            .expect("No ports free")
            .into()
    });

    // static memory is not guaranteed to be dropped.
    // if it isn't be dropped, ``roscore`` do not down and is running after test.
    // Therefore, having weak reference(which cannot live without strong reference).
    static ROSCORE_STATIC: OnceCell<RwLock<Weak<ChildProcessTerminator>>> = OnceCell::new();
    // keep strong reference at least one
    let mut roscore_strong: Option<Arc<ChildProcessTerminator>> = None;

    ONCE.call_once(|| {
        let roscore_terminator = run_roscore(*PORT);

        roscore_strong = Some(Arc::new(roscore_terminator));
        ROSCORE_STATIC
            .set(RwLock::new(Arc::downgrade(
                roscore_strong.as_ref().unwrap(),
            )))
            .unwrap();
        arci_ros::init(init_name);
    });

    if let Some(roscore_arc) = roscore_strong {
        // In current time, ``once_call`` is running.
        // So ``roscore`` is initialized.
        return roscore_arc;
    } else {
        // Try upgrade to ``Arc``, it success if roscore is still alive.
        let roscore_lock = ROSCORE_STATIC.get().unwrap().try_read();
        if let Ok(roscore_read) = roscore_lock {
            let roscore_live = roscore_read.upgrade();
            if let Some(roscore_arc) = roscore_live {
                return roscore_arc;
            }
        }
    }

    // If roscore have already stopped, try to make roscore up.
    // ``roscore`` runner should be only one so we must execute exclusive control.
    // At here, exclusive control is realized by `RwLock::write` .
    let mut roscore_write = ROSCORE_STATIC.get().unwrap().write().unwrap();
    let roscore_live = roscore_write.upgrade();
    if let Some(roscore_arc) = roscore_live {
        return roscore_arc;
    }

    let roscore_terminator = run_roscore(*PORT);
    roscore_strong = Some(Arc::new(roscore_terminator));
    *roscore_write = Arc::downgrade(roscore_strong.as_ref().unwrap());

    roscore_strong.unwrap()
}
