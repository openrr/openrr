// TODO: how to disable entire test?

#[cfg(target_os = "linux")]
mod util;
#[cfg(target_os = "linux")]
use std::collections::HashMap;

#[cfg(target_os = "linux")]
use arci::{gamepad::GamepadEvent, Gamepad};
#[cfg(target_os = "linux")]
use arci_ros::JoyGamepad;
#[cfg(target_os = "linux")]
mod msg {
    rosrust::rosmsg_include!(sensor_msgs / Joy);
}
#[cfg(target_os = "linux")]
use msg::sensor_msgs::Joy;

#[cfg(target_os = "linux")]
#[tokio::test]
async fn test_joy() {
    let _roscore = util::run_roscore_for(util::Language::None, util::Feature::Publisher);

    rosrust::init("test_joy_gamepad");

    let mut button_map = HashMap::new();
    button_map.insert(0, arci::gamepad::Button::West);
    button_map.insert(1, arci::gamepad::Button::South);
    button_map.insert(2, arci::gamepad::Button::East);
    let mut axis_map = HashMap::new();
    axis_map.insert(0, arci::gamepad::Axis::LeftStickX);
    axis_map.insert(1, arci::gamepad::Axis::LeftStickY);
    axis_map.insert(2, arci::gamepad::Axis::RightStickX);
    let pad = JoyGamepad::new("joy", button_map, axis_map);

    let publisher = rosrust::publish::<Joy>("joy", 100).unwrap();
    while arci_ros::is_ok() && publisher.subscriber_count() == 0 {
        println!("waiting subscription");
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    let message1 = Joy {
        axes: vec![0.0, 0.2, 0.0],
        buttons: vec![0, 0, 0],
        ..Joy::default()
    };
    publisher.send(message1).unwrap();
    let message2 = Joy {
        axes: vec![0.0, 0.2, 0.0],
        buttons: vec![0, 0, 1],
        ..Joy::default()
    };
    publisher.send(message2).unwrap();

    let message3 = Joy {
        axes: vec![0.0, 0.2, 0.0],
        buttons: vec![0, 0, 0],
        ..Joy::default()
    };
    publisher.send(message3).unwrap();

    println!("await0");
    let event = pad.next_event().await;
    match event {
        GamepadEvent::AxisChanged(ax, value) => {
            assert_eq!(ax, arci::gamepad::Axis::LeftStickY);
            assert!((value - 0.2).abs() < 0.000001);
        }
        _ => panic!("unexpected {:?}", event),
    }

    println!("await1");
    let event = pad.next_event().await;
    match event {
        GamepadEvent::ButtonPressed(b) => assert_eq!(b, arci::gamepad::Button::East),
        _ => panic!("unexpected {:?}", event),
    }

    println!("await2");
    let event = pad.next_event().await;
    match event {
        GamepadEvent::ButtonReleased(b) => assert_eq!(b, arci::gamepad::Button::East),
        _ => panic!("unexpected {:?}", event),
    }
}
