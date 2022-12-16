use std::{
    sync::Arc,
    time::{Duration, SystemTime},
};

use flume::{Receiver, Sender};
use parking_lot::Mutex;
use rosrust::Time;
type MessageBuffer<T> = Arc<Mutex<Option<T>>>;

fn set_message_buffer<T>(buffer: &MessageBuffer<T>, message: T) {
    buffer.lock().replace(message);
}

fn subscribe_with_message_buffer<T: rosrust::Message>(
    topic: &str,
    queue_size: usize,
) -> (MessageBuffer<T>, rosrust::Subscriber) {
    let buffer: MessageBuffer<T> = Arc::new(Mutex::new(None));
    let buffer_for_callback = buffer.clone();
    let subscriber = rosrust::subscribe(topic, queue_size, move |message: T| {
        set_message_buffer(&buffer_for_callback, message);
    })
    .unwrap();
    (buffer, subscriber)
}

pub struct SubscriberHandler<T> {
    topic: String,
    buffer: MessageBuffer<T>,
    _subscriber: rosrust::Subscriber,
}

impl<T> SubscriberHandler<T>
where
    T: rosrust::Message,
{
    pub fn new(topic: &str, queue_size: usize) -> Self {
        let (buffer, _subscriber) = subscribe_with_message_buffer::<T>(topic, queue_size);
        Self {
            topic: topic.to_string(),
            buffer,
            _subscriber,
        }
    }

    pub fn take(&self) -> Result<Option<T>, arci::Error> {
        Ok(self.buffer.lock().take())
    }

    pub fn get(&self) -> Result<Option<T>, arci::Error> {
        Ok(self.buffer.lock().clone())
    }

    pub fn wait_message(&self, loop_millis: u64) {
        while rosrust::is_ok() && self.get().unwrap().is_none() {
            rosrust::ros_info!("Waiting {}", self.topic);
            std::thread::sleep(Duration::from_millis(loop_millis));
        }
    }
}

pub fn wait_subscriber<T>(publisher: &rosrust::Publisher<T>)
where
    T: rosrust::Message,
{
    let rate = rosrust::rate(10.0);
    while rosrust::is_ok() && publisher.subscriber_count() == 0 {
        rate.sleep();
    }
    // one more to avoid `rostopic echo`
    rate.sleep();
}

type ReceiverStampedBuffer<T> = Arc<Mutex<Receiver<(rosrust::Time, T)>>>;
type SenderStampedBuffer<T> = Arc<Mutex<Sender<(rosrust::Time, T)>>>;

pub struct ServiceHandler<T>
where
    T: rosrust::ServicePair,
{
    request_receiver: ReceiverStampedBuffer<T::Request>,
    response_sender: SenderStampedBuffer<T::Response>,
    _service_name: Arc<Mutex<String>>,
    _server: rosrust::Service,
}

impl<T> ServiceHandler<T>
where
    T: rosrust::ServicePair,
{
    pub fn new(service_name: &str) -> Self {
        let (request_sender, request_receiver) = flume::unbounded::<(rosrust::Time, T::Request)>();
        let (response_sender, response_receiver) =
            flume::unbounded::<(rosrust::Time, T::Response)>();

        let request_sender = Arc::new(Mutex::new(request_sender));
        let request_receiver = Arc::new(Mutex::new(request_receiver));
        let response_sender = Arc::new(Mutex::new(response_sender));
        let response_receiver = Arc::new(Mutex::new(response_receiver));

        let _service_name = Arc::new(Mutex::new(service_name.to_string()));
        let service_name_for_callback = _service_name.clone();

        let _server = rosrust::service::<T, _>(&_service_name.lock(), move |req| {
            let req_time = rosrust::now();
            request_sender.lock().send((req_time, req)).unwrap();
            let (res_time, response) = response_receiver.lock().recv().unwrap();
            if res_time == req_time {
                Ok(response)
            } else {
                Err(format!(
                    "Mismatch time stamp in ServiceHandler for {}",
                    service_name_for_callback.lock()
                ))
            }
        })
        .unwrap();
        Self {
            request_receiver,
            response_sender,
            _service_name,
            _server,
        }
    }

    pub fn get_request(&self, timeout_millis: u32) -> Option<(rosrust::Time, T::Request)> {
        self.request_receiver
            .lock()
            .recv_timeout(Duration::from_millis(timeout_millis as u64))
            .ok()
    }

    pub fn set_response(&self, time: rosrust::Time, res: T::Response) {
        self.response_sender.lock().send((time, res)).unwrap();
    }
}

pub struct ActionResultWait {
    goal_id: String,
    action_result_receiver: Receiver<Result<(), crate::Error>>,
}

impl ActionResultWait {
    pub fn new(
        goal_id: String,
        action_result_receiver: Receiver<Result<(), crate::Error>>,
    ) -> Self {
        Self {
            goal_id,
            action_result_receiver,
        }
    }

    pub fn wait(&mut self, timeout: Duration) -> Result<(), crate::Error> {
        self.action_result_receiver
            .recv_timeout(timeout)
            .map_err(|_| crate::Error::ActionResultTimeout)?
    }

    pub fn goal_id(&self) -> &str {
        &self.goal_id
    }
}

#[macro_export]
macro_rules! define_action_client {
    ($name:ident, $namespace:path, $action_base:expr) => {
        $crate::paste::item! {
            pub struct $name {
                goal_publisher: $crate::rosrust::Publisher<$namespace::[<$action_base ActionGoal>]>,
                _result_subscriber: $crate::rosrust::Subscriber,
                cancel_publisher: $crate::rosrust::Publisher<msg::actionlib_msgs::GoalID>,
                goal_id_to_sender: std::sync::Arc<$crate::parking_lot::Mutex<std::collections::HashMap<String, $crate::flume::Sender<std::result::Result<(), $crate::Error>>>>>
            }
            impl $name {
                pub fn new(base_topic: &str, queue_size: usize) -> Self {
                    use std::sync::Arc;
                    use std::collections::HashMap;
                    use $crate::parking_lot::Mutex;
                    use $crate::Error;
                    use $crate::{flume, rosrust};
                    let goal_topic = format!("{}/goal", base_topic);
                    let cancel_topic = format!("{}/cancel", base_topic);
                    let goal_publisher = rosrust::publish(&goal_topic, queue_size).unwrap();
                    let goal_id_to_sender: Arc<Mutex<HashMap<String, flume::Sender<std::result::Result<(), Error>> >>> = Arc::new(Mutex::new(HashMap::new()));
                    let goal_id_to_sender_cloned = goal_id_to_sender.clone();
                    let _result_subscriber = rosrust::subscribe(
                        &format!("{}/result", base_topic),
                        queue_size, move |result: $namespace::[<$action_base ActionResult>]| {
                            if let Some(sender) = goal_id_to_sender_cloned.lock().remove(&result.status.goal_id.id) {
                                let _ = sender.send(
                                // TODO more detailed error / status handling
                                match result.status.status {
                                    msg::actionlib_msgs::GoalStatus::SUCCEEDED => {
                                        Ok(())
                                    },
                                    msg::actionlib_msgs::GoalStatus::PREEMPTED => {
                                        Err(Error::ActionResultPreempted(format!("{:?}", result)))
                                    },
                                    _ => {
                                        Err(Error::ActionResultNotSuccess(format!("{:?}", result)))
                                    }
                                });
                            }
                        }
                    ).unwrap();
                    let cancel_publisher =
                        rosrust::publish(&cancel_topic, queue_size).unwrap();
                    rosrust::ros_info!("Waiting {} ....", goal_topic);
                    $crate::wait_subscriber(&goal_publisher);
                    rosrust::ros_info!("Waiting {} Done", goal_topic);
                    rosrust::ros_info!("Waiting {} ....", cancel_topic);
                    $crate::wait_subscriber(&cancel_publisher);
                    rosrust::ros_info!("Waiting {} Done", cancel_topic);
                    Self {
                        goal_publisher,
                        _result_subscriber,
                        cancel_publisher,
                        goal_id_to_sender
                     }
                }
                #[allow(dead_code)]
                pub fn send_goal_and_wait(&self, goal: $namespace::[<$action_base Goal>], timeout: std::time::Duration) -> Result<(), $crate::Error> {
                    self.send_goal(goal)?.wait(timeout)
                }
                #[allow(clippy::field_reassign_with_default)]
                pub fn send_goal(&self, goal: $namespace::[<$action_base Goal>]) -> Result<$crate::ActionResultWait, $crate::Error> {
                    use $crate::{flume, rosrust};
                    let mut action_goal = <$namespace::[<$action_base ActionGoal>]>::default();
                    action_goal.goal = goal;
                    let goal_id = format!("{}-{}", rosrust::name(), rosrust::now().seconds());
                    action_goal.goal_id.id = goal_id.clone();
                    let (sender, receiver) = flume::unbounded();
                    self.goal_id_to_sender.lock().insert(goal_id.clone(), sender);
                    if self.goal_publisher.send(action_goal).is_err() {
                        let _ = self.goal_id_to_sender.lock().remove(&goal_id);
                        return Err($crate::Error::ActionGoalSendingFailure);
                    }
                    Ok($crate::ActionResultWait::new(goal_id, receiver))
                }
                #[allow(dead_code)]
                pub fn cancel_goal(&self, goal_id: &str) -> Result<(), $crate::Error> {
                    let mut goal_id_to_sender = self.goal_id_to_sender.lock();
                    if goal_id.is_empty() {
                        goal_id_to_sender.clear();
                    } else {
                        let _ = goal_id_to_sender.remove(goal_id);
                    }
                    if self.cancel_publisher.send(
                    msg::actionlib_msgs::GoalID { id: goal_id.to_owned(), ..Default::default()}).is_err() {
                        return Err($crate::Error::ActionCancelSendingFailure);
                    }
                    Ok(())
                }
                #[allow(dead_code)]
                pub fn cancel_all_goal(&self) -> Result<(), $crate::Error> {
                    self.cancel_goal("")
                }
            }
        }
    };
}

pub fn convert_system_time_to_ros_time(time: &SystemTime) -> Time {
    let ros_now = rosrust::now();
    let system_now = SystemTime::now();

    // compare time to avoid SystemTimeError
    // https://doc.rust-lang.org/std/time/struct.SystemTime.html#method.duration_since
    if system_now < *time {
        Time::from_nanos(
            time.duration_since(system_now).unwrap().as_nanos() as i64 + ros_now.nanos(),
        )
    } else {
        Time::from_nanos(
            ros_now.nanos() - system_now.duration_since(*time).unwrap().as_nanos() as i64,
        )
    }
}

pub fn convert_ros_time_to_system_time(time: &Time) -> SystemTime {
    let ros_now = rosrust::now();
    let system_now = SystemTime::now();
    let ros_time_nanos = time.nanos() as u64;
    let ros_now_nanos = ros_now.nanos() as u64;
    // from_nanos needs u64 as input
    // https://doc.rust-lang.org/stable/std/time/struct.Duration.html#method.from_nanos
    if ros_now_nanos < ros_time_nanos {
        system_now
            .checked_add(std::time::Duration::from_nanos(
                ros_time_nanos - ros_now_nanos,
            ))
            .unwrap()
    } else {
        system_now
            .checked_sub(std::time::Duration::from_nanos(
                ros_now_nanos - ros_time_nanos,
            ))
            .unwrap()
    }
}

/// # subscribe ROS message helper
///
/// using for inspect specific massage type.
/// Message is displayed on screen and sent to `mpsc receiver`
///
/// # Panic!
///
/// If subscriber can't be construct, this function is panic.
/// Or if ``Roscore`` is not up, could be panic.
///
pub fn subscribe_with_channel<T: rosrust::Message>(
    topic_name: &str,
    queue_size: usize,
) -> (flume::Receiver<T>, rosrust::Subscriber) {
    let (tx, rx) = flume::unbounded::<T>();

    let sub = rosrust::subscribe(topic_name, queue_size, move |v: T| {
        tx.send(v).unwrap();
    })
    .unwrap();

    (rx, sub)
}
