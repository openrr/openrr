use arci::Speaker;
use async_trait::async_trait;

pub struct PrintSpeaker {}

impl PrintSpeaker {
    pub fn new() -> Self {
        Self {}
    }
}

impl Default for PrintSpeaker {
    fn default() -> Self {
        PrintSpeaker::new()
    }
}

#[async_trait]
impl Speaker for PrintSpeaker {
    async fn speak(&self, message: &str) -> Result<(), arci::Error> {
        println!("PrintSpeaker: {}", message);
        Ok(())
    }
}
