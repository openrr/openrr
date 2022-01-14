#![cfg(target_os = "linux")]

mod util;

use std::{result::Result::Ok, time::SystemTime};

use arci_ros::{
    convert_ros_time_to_system_time, convert_system_time_to_ros_time, subscribe_with_channel,
};
use util::run_roscore_and_rosrust_init_once;

#[test]
fn test_convert_ros_time_to_system_time() {
    let _roscore = run_roscore_and_rosrust_init_once("test_convert_ros_time_to_system_time");

    const ALLOWABLE_ERROR_NANOSECONDS: u128 = 1_000_000;

    let ros_diff = rosrust::Duration::from_seconds(1);
    let system_time_diff = std::time::Duration::from_secs(1);

    let ros_time = rosrust::now();
    let system_time = SystemTime::now();
    let system_time_1second_ago = convert_ros_time_to_system_time(&(ros_time - ros_diff));
    let system_time_1second = convert_ros_time_to_system_time(&(ros_time + ros_diff));

    let system_time_actual_diff = system_time.duration_since(system_time_1second_ago);
    assert!(system_time_actual_diff.is_ok());
    let system_time_actual_diff_nanos = system_time_actual_diff.unwrap().as_nanos();
    assert!(
        system_time_diff.as_nanos() - ALLOWABLE_ERROR_NANOSECONDS < system_time_actual_diff_nanos
            && system_time_actual_diff_nanos
                < system_time_diff.as_nanos() + ALLOWABLE_ERROR_NANOSECONDS,
        "actual {} expected {} < < {}",
        system_time_actual_diff_nanos,
        system_time_diff.as_nanos() - ALLOWABLE_ERROR_NANOSECONDS,
        system_time_diff.as_nanos() + ALLOWABLE_ERROR_NANOSECONDS
    );

    let system_time_actual_diff = system_time_1second.duration_since(system_time);
    assert!(system_time_actual_diff.is_ok());
    let system_time_actual_diff_nanos = system_time_actual_diff.unwrap().as_nanos();
    assert!(
        system_time_diff.as_nanos() - ALLOWABLE_ERROR_NANOSECONDS < system_time_actual_diff_nanos
            && system_time_actual_diff_nanos
                < system_time_diff.as_nanos() + ALLOWABLE_ERROR_NANOSECONDS,
        "actual {} expected {} < < {}",
        system_time_actual_diff_nanos,
        system_time_diff.as_nanos() - ALLOWABLE_ERROR_NANOSECONDS,
        system_time_diff.as_nanos() + ALLOWABLE_ERROR_NANOSECONDS
    );
}

#[flaky_test::flaky_test]
fn test_convert_system_time_to_ros_time() {
    let _roscore =
        run_roscore_and_rosrust_init_once(&"test_convert_system_time_to_ros_time".to_owned());

    const ALLOWABLE_ERROR: f64 = 1e-5;
    let system_time_diff = std::time::Duration::from_secs(1);

    let ros_time = rosrust::now();
    let system_time = SystemTime::now();

    let ros_time_1second_ago =
        convert_system_time_to_ros_time(&system_time.checked_sub(system_time_diff).unwrap());
    let ros_time_1second =
        convert_system_time_to_ros_time(&system_time.checked_add(system_time_diff).unwrap());
    let actual_diff = ((ros_time - ros_time_1second_ago).seconds() - 1.0).abs();
    assert!(
        actual_diff < ALLOWABLE_ERROR,
        "actual {} expected {}",
        actual_diff,
        ALLOWABLE_ERROR
    );
    let actual_diff = ((ros_time_1second - ros_time).seconds() - 1.0).abs();
    assert!(
        actual_diff < ALLOWABLE_ERROR,
        "actual {} expected {}",
        actual_diff,
        ALLOWABLE_ERROR
    );
}

#[flaky_test::flaky_test]
fn test_subscribe_with_channel() {
    use arci::{BaseVelocity, MoveBase};
    use arci_ros::{msg::geometry_msgs::Twist, RosCmdVelMoveBase};
    use assert_approx_eq::assert_approx_eq;

    println!("test subscriber helper is running!");
    let topic = "sub_test_twist".to_owned();
    let _roscore = run_roscore_and_rosrust_init_once(&"test_subscribe_with_channel".to_owned());

    let (rx, _sub) = subscribe_with_channel::<Twist>(&topic, 1);
    let c = RosCmdVelMoveBase::new(&topic);
    let mut vel = BaseVelocity::default();
    const NUMBER_OF_TEST_MESSAGES: usize = 50;

    // publish message
    for count in 0..NUMBER_OF_TEST_MESSAGES {
        vel.x = 0.001 * (count as f64);
        c.send_velocity(&vel).unwrap();
        std::thread::sleep(std::time::Duration::from_millis(100));
        println!("{count}, {vel:?}");
    }

    // subscribe(receiving from mpsc)
    let mut rv_count = 0_usize;
    while let Ok(rv) = rx.recv() {
        assert_approx_eq!(rv.linear.x, 0.001 * (rv_count as f64));
        println!("subscribe(receiving from mpsc): {rv:?}");

        rv_count += 1;
        if rv_count >= NUMBER_OF_TEST_MESSAGES {
            break;
        }
    }
}
