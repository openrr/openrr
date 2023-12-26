use std::{
    sync::{Arc, RwLock},
    time::{Duration, Instant, SystemTime},
};

use futures::future::FutureExt;
use ros2_client::{action, builtin_interfaces::Time, ros2};
use serde::de::DeserializeOwned;

const BILLION: u128 = 1_000_000_000;

pub(crate) fn subscribe_thread<M: DeserializeOwned + Send + 'static, U: Send + Sync + 'static>(
    subscriber: ros2_client::Subscription<M>,
    buf: Arc<RwLock<U>>,
    mut f: impl FnMut(M) -> U + Send + 'static,
) {
    tokio::spawn(async move {
        while Arc::strong_count(&buf) > 1 {
            if let Some((val, _info)) = subscriber.async_take().now_or_never().transpose().unwrap()
            {
                let res = f(val);
                *buf.write().unwrap() = res;
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });
}

pub(crate) fn subscribe_one<M: DeserializeOwned + Send + 'static>(
    subscriber: &mut ros2_client::Subscription<M>,
    timeout: Duration,
) -> Option<M> {
    // https://github.com/openrr/openrr/pull/501#discussion_r746183161
    std::thread::scope(|s| {
        s.spawn(|| {
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(async move {
                    let start = Instant::now();
                    let mut prev = None;
                    loop {
                        if let Some((val, _info)) =
                            subscriber.async_take().now_or_never().transpose().unwrap()
                        {
                            prev = Some(val);
                            continue;
                        }
                        if prev.is_some() {
                            return prev;
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

pub(crate) fn topic_qos() -> ros2::QosPolicies {
    ros2::QosPolicyBuilder::new()
        .durability(ros2::policy::Durability::Volatile)
        .liveliness(ros2::policy::Liveliness::Automatic {
            lease_duration: ros2::Duration::INFINITE,
        })
        .reliability(ros2::policy::Reliability::Reliable {
            max_blocking_time: ros2::Duration::from_millis(100),
        })
        .history(ros2::policy::History::KeepLast { depth: 1 })
        .build()
}

pub(crate) fn service_qos() -> ros2::QosPolicies {
    ros2::QosPolicyBuilder::new()
        .reliability(ros2::policy::Reliability::Reliable {
            max_blocking_time: ros2::Duration::from_millis(100),
        })
        .history(ros2::policy::History::KeepLast { depth: 1 })
        .build()
}

pub(crate) fn action_client_qos() -> action::ActionClientQosPolicies {
    let service_qos = service_qos();
    action::ActionClientQosPolicies {
        goal_service: service_qos.clone(),
        result_service: service_qos.clone(),
        cancel_service: service_qos.clone(),
        feedback_subscription: service_qos.clone(),
        status_subscription: service_qos,
    }
}

pub(crate) fn action_server_qos() -> action::ActionServerQosPolicies {
    let service_qos = service_qos();
    let publisher_qos = ros2::QosPolicyBuilder::new()
        .reliability(ros2::policy::Reliability::Reliable {
            max_blocking_time: ros2::Duration::from_millis(100),
        })
        .history(ros2::policy::History::KeepLast { depth: 1 })
        .durability(ros2::policy::Durability::TransientLocal)
        .build();
    action::ActionServerQosPolicies {
        goal_service: service_qos.clone(),
        result_service: service_qos.clone(),
        cancel_service: service_qos.clone(),
        feedback_publisher: publisher_qos.clone(),
        status_publisher: publisher_qos,
    }
}

pub fn convert_system_time_to_ros2_time(time: &SystemTime) -> Time {
    let ros_now = Time::now();
    let system_now = SystemTime::now();

    let nano = if system_now < *time {
        time.duration_since(system_now).unwrap().as_nanos() + time_as_nanos(ros_now)
    } else {
        time_as_nanos(ros_now) - system_now.duration_since(*time).unwrap().as_nanos()
    };

    Time {
        sec: (nano / BILLION) as i32,
        nanosec: (nano % BILLION) as u32,
    }
}

pub fn convert_ros2_time_to_system_time(time: &Time) -> SystemTime {
    let ros_now = Time::now();
    let system_now = SystemTime::now();
    let ros_time_nanos = time_as_nanos(*time);
    let ros_now_nanos = time_as_nanos(ros_now);

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

fn time_as_nanos(time: Time) -> u128 {
    time.sec as u128 * BILLION + time.nanosec as u128
}
