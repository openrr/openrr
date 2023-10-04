use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::{Duration, SystemTime},
};

use futures::{
    future::Future,
    stream::{Stream, StreamExt},
};
use r2r::builtin_interfaces::msg::Time;

const BILLION: u128 = 1_000_000_000;

// https://github.com/openrr/openrr/pull/501#discussion_r746183161
pub(crate) fn spawn_blocking<T: Send + 'static>(
    future: impl Future<Output = T> + Send + 'static,
) -> thread::JoinHandle<T> {
    std::thread::spawn(move || {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(future)
    })
}

pub(crate) async fn wait(is_done: Arc<AtomicBool>) {
    loop {
        if is_done.load(Ordering::Relaxed) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

pub(crate) async fn subscribe_one<T: Send + 'static>(
    mut subscriber: impl Stream<Item = T> + Send + Unpin + 'static,
) -> Result<Option<T>, tokio::task::JoinError> {
    let is_done = Arc::new(AtomicBool::new(false));
    let is_done_clone = is_done.clone();
    let handle = tokio::spawn(async move {
        let next = subscriber.next().await;
        is_done.store(true, Ordering::Relaxed);
        next
    });
    wait(is_done_clone).await;
    handle.await
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
            .checked_add(std::time::Duration::from_nanos(
                (ros_time_nanos - ros_now_nanos) as u64,
            ))
            .unwrap()
    } else {
        system_now
            .checked_sub(std::time::Duration::from_nanos(
                (ros_now_nanos - ros_time_nanos) as u64,
            ))
            .unwrap()
    }
}
