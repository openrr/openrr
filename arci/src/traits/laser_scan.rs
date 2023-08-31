use auto_impl::auto_impl;

use crate::error::Error;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Scan2D {
    /// The angle (in radians) where the scan starts, typically 0 for positive X-axis.
    pub angle_min: f64,
    /// The angle (in radians) where the scan ends, typically positive for CCW scans.
    pub angle_max: f64,
    /// The angular difference (in radians) between consecutive scans.
    pub angle_increment: f64,
    /// The time difference (in seconds) between consecutive measurements.
    pub time_increment: f64,
    /// The time (in seconds) it takes for the laser to complete one scan.
    pub scan_time: f64,
    /// The minimum range (in meters) of valid distance measurements.
    pub range_min: f64,
    /// The maximum range (in meters) of valid distance measurements.
    pub range_max: f64,
    /// An array of distance measurements (in meters) for each angle.
    pub ranges: Vec<f64>,
    /// (Optional) An array of intensity values for each angle, not supported by all LIDAR sensors.
    pub intensities: Vec<f64>,
}

#[auto_impl(Box, Arc)]
pub trait LaserScan2D: Send + Sync {
    fn current_scan(&self) -> Result<Scan2D, Error>;
}
