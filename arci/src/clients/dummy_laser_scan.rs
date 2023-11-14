use std::sync::Mutex;

use crate::{Error, LaserScan2D, Scan2D};

#[derive(Debug, Default)]
pub struct DummyLaserScan2D {
    scan: Mutex<Scan2D>,
}

impl DummyLaserScan2D {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_scan(&self, scan: Scan2D) {
        *self.scan.lock().unwrap() = scan;
    }
}

impl LaserScan2D for DummyLaserScan2D {
    fn current_scan(&self) -> Result<Scan2D, Error> {
        Ok(self.scan.lock().unwrap().clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scan() {
        let laser_scan = DummyLaserScan2D::new();
        assert_eq!(laser_scan.current_scan().unwrap(), Scan2D::default());
        laser_scan.set_scan(Scan2D {
            angle_min: 1.0,
            angle_max: 2.0,
            ..Default::default()
        });
        assert_eq!(
            laser_scan.current_scan().unwrap(),
            Scan2D {
                angle_min: 1.0,
                angle_max: 2.0,
                ..Default::default()
            }
        );
    }
}
