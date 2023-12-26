mod shared;

use std::time::Duration;

use arci::{LaserScan2D, Scan2D};
use arci_ros2::{
    msg::{sensor_msgs::LaserScan, std_msgs::Header},
    Ros2LaserScan2D,
};
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
    let laser_scan_topic = node.create_topic::<LaserScan>(LASER_SCAN_TOPIC).unwrap();
    let scan_publisher = node
        .create_publisher::<LaserScan>(&laser_scan_topic)
        .unwrap();

    tokio::spawn(async move {
        for _ in 0..2 {
            scan_publisher
                .publish(LaserScan {
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
