use arci::gamepad::{Gamepad, GamepadEvent};
use arci_gamepad_gilrs::*;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt::init();
    let g = GilGamepad::new(0, Map::default(), FlagType::Event);
    loop {
        let ev = g.next_event().await;
        match ev {
            GamepadEvent::Unknown => break,
            _ => {
                println!("Result = {ev:?}");
            }
        }
    }
}
