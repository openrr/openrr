use std::collections::HashMap;

use anyhow::Error;
use arci::Gamepad;
use arci_ros::JoyGamepad;

#[tokio::main]
async fn main() -> Result<(), Error> {
    arci_ros::init("joy_gamepad");
    let mut button_map = HashMap::new();
    button_map.insert(0, arci::gamepad::Button::West);
    button_map.insert(1, arci::gamepad::Button::South);
    button_map.insert(2, arci::gamepad::Button::East);
    button_map.insert(3, arci::gamepad::Button::North);
    let mut axis_map = HashMap::new();
    axis_map.insert(0, arci::gamepad::Axis::LeftStickX);
    axis_map.insert(1, arci::gamepad::Axis::LeftStickY);
    axis_map.insert(2, arci::gamepad::Axis::RightStickX);
    axis_map.insert(5, arci::gamepad::Axis::RightStickY);
    axis_map.insert(6, arci::gamepad::Axis::DPadY);
    axis_map.insert(7, arci::gamepad::Axis::DPadX);
    let pad = JoyGamepad::new("joy", button_map, axis_map);
    while arci_ros::is_ok() {
        let event = pad.next_event().await;
        println!("{:?}", event);
    }
    Ok(())
}
