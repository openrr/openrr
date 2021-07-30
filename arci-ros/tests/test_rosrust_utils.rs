#![cfg(target_os = "linux")]

mod util;
use std::time::SystemTime;

use arci_ros::{convert_ros_time_to_system_time, convert_system_time_to_ros_time};

#[test]
fn test() {
    let _ros_core = util::run_roscore_for(util::Language::None, util::Feature::Publisher);
    // to avoid calling rosrust::init multiple times, call all tests from this function.
    rosrust::init("test_rosrust_utils");
    test_convert_ros_time_to_system_time();
    test_convert_system_time_to_ros_time();
}
fn test_convert_ros_time_to_system_time() {
    const ALLOWABLE_ERROR_NANOSECONDS: u128 = 1_000;

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
        system_time_diff.as_nanos() - ALLOWABLE_ERROR_NANOSECONDS,
        system_time_actual_diff_nanos,
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
        system_time_diff.as_nanos() - ALLOWABLE_ERROR_NANOSECONDS,
        system_time_actual_diff_nanos,
        system_time_diff.as_nanos() + ALLOWABLE_ERROR_NANOSECONDS
    );
}

fn test_convert_system_time_to_ros_time() {
    const ALLOWABLE_ERROR: f64 = 1.0 / 1_000_000.0;
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
