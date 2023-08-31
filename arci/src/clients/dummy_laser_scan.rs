use crate::{Error, LaserScan2D, Scan2D};

#[derive(Debug, Clone, Default)]
pub struct DummyLaserScan2D;

impl DummyLaserScan2D {
    pub fn new() -> Self {
        Self
    }
}

impl LaserScan2D for DummyLaserScan2D {
    fn current_scan(&self) -> Result<Scan2D, Error> {
        Ok(Scan2D::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scan() {
        let laser_scan = DummyLaserScan2D::new();
        assert_eq!(laser_scan.current_scan().unwrap(), Scan2D::default());
    }
}
