/// RAII sleep
pub struct ScopedSleep {
    target_time: std::time::Instant,
}

impl ScopedSleep {
    /// Create sleep
    ///
    /// # Example
    /// ```
    /// let now = std::time::Instant::now();
    /// {
    ///     // Do not use `let _ = ..` here because it immediately drops ScopedSleep.
    ///     let _guard = openrr_sleep::ScopedSleep::new(std::time::Duration::from_millis(100));
    ///     // not sleep yet
    ///     assert!(now.elapsed() < std::time::Duration::from_millis(20));
    /// }
    /// // slept
    /// assert!(now.elapsed() > std::time::Duration::from_millis(20));
    /// ```
    pub fn new(sleep_duration: std::time::Duration) -> Self {
        ScopedSleep {
            target_time: std::time::Instant::now() + sleep_duration,
        }
    }

    /// Create sleep from float secs value
    ///
    /// # Example
    /// ```
    /// let now = std::time::Instant::now();
    /// {
    ///     // Do not use `let _ = ..` here because it immediately drops ScopedSleep.
    ///     let _guard = openrr_sleep::ScopedSleep::from_secs(0.1);
    ///     // not sleep yet
    ///     assert!(now.elapsed() < std::time::Duration::from_millis(10));
    /// }
    /// // slept
    /// assert!(now.elapsed() > std::time::Duration::from_millis(10));
    /// ```
    pub fn from_secs(sleep_duration_sec: f64) -> Self {
        Self::new(std::time::Duration::from_secs_f64(sleep_duration_sec))
    }
}

impl Drop for ScopedSleep {
    fn drop(&mut self) {
        let now = std::time::Instant::now();
        if now < self.target_time {
            std::thread::sleep(self.target_time - now);
        }
    }
}
