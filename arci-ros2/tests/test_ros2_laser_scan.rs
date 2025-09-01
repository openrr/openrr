#![cfg(feature = "ros2")]

mod shared;

use std::time::Duration;

use arci::{LaserScan2D, Scan2D};
use arci_ros2::{Ros2LaserScan2D, r2r};
use r2r::{QosProfile, sensor_msgs::msg::LaserScan, std_msgs::msg::Header};
use shared::*;

const LASER_SCAN_TOPIC: &str = "/scan";

const ANGLE_MAX: f32 = std::f32::consts::FRAC_PI_2;
const ANGLE_INCREMENT: f32 = 0.1;
const TIME_INCREMENT: f32 = 0.01;
const SCAN_TIME: f32 = 123.4;
const RANGE_MIN: f32 = 0.1;
const RANGE_MAX: f32 = 12.0;

#[tokio::test(flavor = "multi_thread")]
async fn test_laser_scan() {
    let node = test_node();
    let scan_publisher = node
        .r2r()
        .create_publisher::<LaserScan>(LASER_SCAN_TOPIC, QosProfile::default())
        .unwrap();

    tokio::spawn(async move {
        loop {
            scan_publisher
                .publish(&LaserScan {
                    header: Header::default(),
                    angle_min: 0.,
                    angle_max: ANGLE_MAX,
                    angle_increment: ANGLE_INCREMENT,
                    time_increment: TIME_INCREMENT,
                    scan_time: SCAN_TIME,
                    range_min: RANGE_MIN,
                    range_max: RANGE_MAX,
                    ranges: vec![1.; (ANGLE_MAX / ANGLE_INCREMENT) as usize],
                    intensities: vec![],
                })
                .unwrap();
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });

    node.run_spin_thread(Duration::from_millis(100));
    let client = Ros2LaserScan2D::new(node, LASER_SCAN_TOPIC).unwrap();

    let current_scan = client.current_scan().unwrap();

    assert_eq!(
        current_scan,
        Scan2D {
            angle_min: 0.,
            angle_max: ANGLE_MAX as f64,
            angle_increment: ANGLE_INCREMENT as f64,
            time_increment: TIME_INCREMENT as f64,
            scan_time: SCAN_TIME as f64,
            range_min: RANGE_MIN as f64,
            range_max: RANGE_MAX as f64,
            ranges: vec![1.; (ANGLE_MAX / ANGLE_INCREMENT) as usize],
            intensities: vec![]
        }
    )
}
