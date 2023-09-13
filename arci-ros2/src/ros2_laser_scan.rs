use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

use arci::*;
use futures::StreamExt;
use parking_lot::Mutex;
use r2r::{sensor_msgs::msg::LaserScan, QosProfile};
use serde::{Deserialize, Serialize};

use crate::utils;

/// Implement arci::LaserScan2D for ROS2
pub struct Ros2LaserScan2D {
    node: Arc<Mutex<r2r::Node>>,
    laser_scan_topic_name: String,
}

impl Ros2LaserScan2D {
    /// Creates a new `Ros2LaserScan2D`.
    pub fn new(ctx: r2r::Context, laser_scan_topic_name: &str) -> Self {
        let node = r2r::Node::create(ctx, "openrr_ros2_laser_scan_node", "arci_ros2").unwrap();

        Self {
            node: Arc::new(Mutex::new(node)),
            laser_scan_topic_name: laser_scan_topic_name.to_owned(),
        }
    }
}

impl LaserScan2D for Ros2LaserScan2D {
    fn current_scan(&self) -> Result<arci::Scan2D, arci::Error> {
        let node_clone = self.node.clone();

        let mut scan_subscriber = node_clone
            .lock()
            .subscribe::<LaserScan>(&self.laser_scan_topic_name, QosProfile::default())
            .unwrap();

        let is_done = Arc::new(AtomicBool::new(false));
        let is_done_clone = is_done.clone();

        let subscribed_scan = utils::spawn_blocking(async move {
            let handle = tokio::spawn(async move {
                let next = scan_subscriber.next().await;
                is_done_clone.store(true, Ordering::SeqCst);
                next
            });
            loop {
                if is_done.load(Ordering::SeqCst) {
                    break;
                }
                node_clone.lock().spin_once(Duration::from_millis(100));
                tokio::task::yield_now().await;
            }
            handle.await.unwrap()
        })
        .join()
        .unwrap();

        let current_scan = match subscribed_scan {
            Some(msg) => Scan2D {
                angle_min: msg.angle_min as f64,
                angle_max: msg.angle_max as f64,
                angle_increment: msg.angle_increment as f64,
                time_increment: msg.time_increment as f64,
                scan_time: msg.scan_time as f64,
                range_min: msg.range_min as f64,
                range_max: msg.range_max as f64,
                ranges: msg.ranges.iter().map(|&v| v as f64).collect::<Vec<f64>>(),
                intensities: msg
                    .intensities
                    .iter()
                    .map(|&v| v as f64)
                    .collect::<Vec<f64>>(),
            },
            None => {
                return Err(Error::Connection {
                    message: format!("Failed to get scan from {}", self.laser_scan_topic_name),
                });
            }
        };

        Ok(current_scan)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
/// Config for Ros2LaserScanConfig
pub struct Ros2LaserScan2DConfig {
    /// topic name for LaserScan
    pub topic: String,
}
