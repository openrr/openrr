use arci::Speaker;

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

impl Speaker for PrintSpeaker {
    fn speak(&self, message: &str) {
        println!("PrintSpeaker: {}", message);
    }
}
