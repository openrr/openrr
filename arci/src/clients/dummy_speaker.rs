use std::sync::Mutex;

use crate::{error::Error, traits::Speaker, WaitFuture};

/// Dummy Speaker for debug or tests.
#[derive(Debug, Default)]
pub struct DummySpeaker {
    pub message: Mutex<String>,
}

impl DummySpeaker {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn current_message(&self) -> String {
        self.message.lock().unwrap().clone()
    }
}

impl Speaker for DummySpeaker {
    fn speak(&self, message: &str) -> Result<WaitFuture<'static>, Error> {
        *self.message.lock().unwrap() = message.to_string();
        Ok(WaitFuture::ready())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_set() {
        let speaker = DummySpeaker::new();

        assert_eq!(speaker.current_message(), "");
        speaker.speak("abc").unwrap().await.unwrap();
        assert_eq!(speaker.current_message(), "abc");
    }

    #[test]
    fn test_set_no_wait() {
        let speaker = DummySpeaker::new();

        assert_eq!(speaker.current_message(), "");
        let _ = speaker.speak("abc").unwrap();
        assert_eq!(speaker.current_message(), "abc");
    }
}
