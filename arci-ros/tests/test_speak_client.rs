#[cfg(target_os = "linux")]
use arci::Speaker;
mod msg {
    rosrust::rosmsg_include!(std_msgs / String);
}
use arci_ros::subscribe_with_channel;
use msg::std_msgs;
mod util;
use util::run_roscore_and_rosrust_init_once;

const MESSAGE: &str = "Good morning!";

#[test]
fn test_speak_client() {
    println!("arci ros speak client test");

    // Setup for ROS
    let _roscore = run_roscore_and_rosrust_init_once("arci_ros_speak_client_test");

    let topic_name = String::from("test_speak");
    let (rx, _sub) = subscribe_with_channel::<std_msgs::String>(&topic_name, 2);

    let speaker = arci_ros::RosEspeakClient::new(&topic_name);

    std::thread::spawn(move || {
        // Wait for starting up receiver
        std::thread::sleep(std::time::Duration::from_millis(100));

        let _wait_future = speaker.speak(MESSAGE).unwrap();
        println!("[Speaker]: {MESSAGE:?}");
    });

    if let Ok(rv) = rx.recv() {
        assert_eq!(rv.data, MESSAGE);
        println!("[Receiver]: {:?}", rv.data);
    }
}
