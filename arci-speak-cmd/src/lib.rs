#![doc = include_str!("../README.md")]
#![warn(missing_docs, rust_2018_idioms)]
// This lint is unable to correctly determine if an atomic is sufficient to replace the mutex use.
// https://github.com/rust-lang/rust-clippy/issues/4295
#![allow(clippy::mutex_atomic)]

use std::{io, process::Command};

use arci::{Speaker, WaitFuture};

/// A [`Speaker`] implementation using a local command.
///
/// Currently, this uses the following command:
///
/// - On macOS, use `say` command.
/// - On Windows, call [SAPI] via PowerShell.
/// - On others, use `espeak` command.
///
/// **Disclaimer**: These commands might change over time.
///
/// [SAPI]: https://en.wikipedia.org/wiki/Microsoft_Speech_API
#[derive(Debug, Default)]
#[non_exhaustive]
pub struct LocalCommand {}

impl LocalCommand {
    /// Creates a new `LocalCommand`.
    pub fn new() -> Self {
        Self::default()
    }
}

impl Speaker for LocalCommand {
    fn speak(&self, message: &str) -> Result<WaitFuture, arci::Error> {
        let (sender, receiver) = tokio::sync::oneshot::channel();
        let message = message.to_string();

        std::thread::spawn(move || {
            let res = run_local_command(&message).map_err(|e| arci::Error::Other(e.into()));
            let _ = sender.send(res);
        });

        Ok(WaitFuture::new(async move {
            receiver.await.map_err(|e| arci::Error::Other(e.into()))?
        }))
    }
}

#[cfg(not(windows))]
fn run_local_command(message: &str) -> io::Result<()> {
    #[cfg(not(target_os = "macos"))]
    const CMD_NAME: &str = "espeak";
    #[cfg(target_os = "macos")]
    const CMD_NAME: &str = "say";

    let mut cmd = Command::new(CMD_NAME);
    let status = cmd.arg(message).status()?;

    if status.success() {
        Ok(())
    } else {
        Err(io::Error::new(
            io::ErrorKind::Other,
            format!("failed to run `{CMD_NAME}` with message {message:?}"),
        ))
    }
}

#[cfg(windows)]
fn run_local_command(message: &str) -> io::Result<()> {
    // TODO: Ideally, it would be more efficient to use SAPI directly via winapi or something.
    // https://stackoverflow.com/questions/1040655/ms-speech-from-command-line
    let cmd = format!("PowerShell -Command \"Add-Type –AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('{message}');\"");
    let status = Command::new("powershell").arg(cmd).status()?;

    if status.success() {
        Ok(())
    } else {
        Err(io::Error::new(
            io::ErrorKind::Other,
            format!("failed to run `powershell` with message {message:?}"),
        ))
    }
}
