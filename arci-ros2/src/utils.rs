use std::{
    sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant, SystemTime},
};

use futures::{
    future::FutureExt,
    stream::{Stream, StreamExt},
};
use r2r::builtin_interfaces::msg::Time;

const BILLION: u128 = 1_000_000_000;

// TODO: timeout
pub(crate) async fn wait(is_done: Arc<AtomicBool>) {
    loop {
        if is_done.load(Ordering::Relaxed) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

pub(crate) fn subscribe_thread<T: Send + 'static, U: Send + Sync + 'static>(
    mut subscriber: impl Stream<Item = T> + Send + Unpin + 'static,
    buf: Arc<RwLock<U>>,
    mut f: impl FnMut(T) -> U + Send + 'static,
) {
    tokio::spawn(async move {
        while Arc::strong_count(&buf) > 1 {
            if let Some(val) = subscriber.next().await {
                let res = f(val);
                *buf.write().unwrap() = res;
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });
}

pub(crate) fn subscribe_one<T: Send>(
    mut subscriber: impl Stream<Item = T> + Send + Unpin,
    timeout: Duration,
) -> Option<T> {
    // https://github.com/openrr/openrr/pull/501#discussion_r746183161
    std::thread::scope(|s| {
        s.spawn(|| {
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(async move {
                    let start = Instant::now();
                    loop {
                        if let Some(v) = subscriber.next().now_or_never() {
                            return v;
                        }
                        if start.elapsed() > timeout {
                            return None; // timeout
                        }
                        tokio::time::sleep(Duration::from_millis(100)).await;
                    }
                })
        })
        .join()
        .unwrap()
    })
}

pub fn convert_system_time_to_ros2_time(time: &SystemTime) -> Time {
    let mut ros_clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let ros_now = ros_clock.get_now().unwrap();
    let system_now = SystemTime::now();

    let nano = if system_now < *time {
        time.duration_since(system_now).unwrap().as_nanos() + ros_now.as_nanos()
    } else {
        ros_now.as_nanos() - system_now.duration_since(*time).unwrap().as_nanos()
    };

    Time {
        sec: (nano / BILLION) as i32,
        nanosec: (nano % BILLION) as u32,
    }
}

pub fn convert_ros2_time_to_system_time(time: &Time) -> SystemTime {
    let mut ros_clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let ros_now = ros_clock.get_now().unwrap();
    let system_now = SystemTime::now();
    let ros_time_nanos = time.sec as u128 * BILLION + time.nanosec as u128;
    let ros_now_nanos = ros_now.as_nanos();

    if ros_now_nanos < ros_time_nanos {
        system_now
            .checked_add(Duration::from_nanos(
                (ros_time_nanos - ros_now_nanos) as u64,
            ))
            .unwrap()
    } else {
        system_now
            .checked_sub(Duration::from_nanos(
                (ros_now_nanos - ros_time_nanos) as u64,
            ))
            .unwrap()
    }
}
