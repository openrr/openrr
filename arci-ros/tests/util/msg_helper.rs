#![cfg(target_os = "linux")]

use std::sync::mpsc;

/// # subscribe ROS message helper
///
/// using for inspect specific massage type.
/// Message is displayed on screen and sent to ``mpsc receiver``
///
/// # Panic!
///
/// If subscriber can't be construct, this function is panic.
///
pub(crate) fn subscribe_helper<T: rosrust::Message>(
    topic_name: &str,
) -> (mpsc::Receiver<T>, rosrust::Subscriber) {
    let (tx, rx) = mpsc::channel::<T>();

    let sub = rosrust::subscribe(topic_name, 1, move |v: T| {
        println!("callback {:?}", v);
        tx.send(v).unwrap();
    })
    .unwrap();

    (rx, sub)
}
