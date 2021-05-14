use arci::gamepad::{Gamepad, GamepadEvent};
use arci_gamepad_keyboard::*;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt::init();
    let g = KeyboardGamepad::new();
    loop {
        let ev = g.next_event().await;
        match ev {
            GamepadEvent::Unknown => {}
            _ => {
                println!("Result = {:?}", ev);
            }
        }
    }
}
