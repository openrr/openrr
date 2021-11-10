#[cfg(unix)]
use arci::gamepad::{Gamepad, GamepadEvent};
#[cfg(unix)]
use arci_gamepad_keyboard::*;

#[cfg(unix)]
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

#[cfg(windows)]
fn main() {
    println!("This example does not work on Windows");
}
