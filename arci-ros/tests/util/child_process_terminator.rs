use std::process::{Child, Command, Stdio};

use nix::{
    sys::signal::{Signal, kill},
    unistd::Pid,
};

#[must_use]
#[derive(Debug)]
pub(crate) struct ChildProcessTerminator(pub(crate) Child);

impl ChildProcessTerminator {
    #[track_caller]
    pub(crate) fn spawn(command: &mut Command) -> ChildProcessTerminator {
        command
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null());
        ChildProcessTerminator(command.spawn().unwrap())
    }

    #[allow(dead_code)]
    pub(crate) fn spawn_example(command: &mut Command) -> ChildProcessTerminator {
        assert!(
            Command::new("cargo")
                .arg("build")
                .arg("--all-targets")
                .output()
                .unwrap()
                .status
                .success()
        );

        Self::spawn(command)
    }

    #[allow(dead_code)]
    pub(crate) fn spawn_example_bench(command: &mut Command) -> ChildProcessTerminator {
        assert!(
            Command::new("cargo")
                .arg("build")
                .arg("--all-targets")
                .arg("--release")
                .output()
                .unwrap()
                .status
                .success()
        );

        Self::spawn(command)
    }
}

impl Drop for ChildProcessTerminator {
    fn drop(&mut self) {
        let pid = Pid::from_raw(self.0.id() as i32);
        kill(pid, Signal::SIGINT).unwrap();
    }
}
