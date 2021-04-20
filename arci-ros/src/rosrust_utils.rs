use std::sync::{
    mpsc::{Receiver, Sender},
    Arc, Mutex,
};

type MessageBuffer<T> = Arc<Mutex<Option<T>>>;

fn set_message_buffer<T>(buffer: &MessageBuffer<T>, message: T) {
    buffer.lock().unwrap().replace(message);
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
        Ok(self
            .buffer
            .lock()
            .map_err(|e| anyhow::anyhow!("Failed to lock buffer for {} : {}", self.topic, e))?
            .take())
    }

    pub fn get(&self) -> Result<Option<T>, arci::Error> {
        Ok(self
            .buffer
            .lock()
            .map_err(|e| anyhow::anyhow!("Failed to lock buffer for {} : {}", self.topic, e))?
            .clone())
    }

    pub fn wait_message(&self, loop_millis: u64) {
        while rosrust::is_ok() && self.get().unwrap().is_none() {
            rosrust::ros_info!("Waiting {}", self.topic);
            std::thread::sleep(std::time::Duration::from_millis(loop_millis));
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
        let (request_sender, request_receiver) =
            std::sync::mpsc::channel::<(rosrust::Time, T::Request)>();
        let (response_sender, response_receiver) =
            std::sync::mpsc::channel::<(rosrust::Time, T::Response)>();

        let request_sender = Arc::new(Mutex::new(request_sender));
        let request_receiver = Arc::new(Mutex::new(request_receiver));
        let response_sender = Arc::new(Mutex::new(response_sender));
        let response_receiver = Arc::new(Mutex::new(response_receiver));

        let _service_name = Arc::new(Mutex::new(service_name.to_string()));
        let service_name_for_callback = _service_name.clone();

        let _server = rosrust::service::<T, _>(&_service_name.lock().unwrap(), move |req| {
            let req_time = rosrust::now();
            request_sender
                .lock()
                .unwrap()
                .send((req_time, req))
                .unwrap();
            let (res_time, response) = response_receiver.lock().unwrap().recv().unwrap();
            if res_time == req_time {
                Ok(response)
            } else {
                Err(format!(
                    "Mismatch time stamp in ServiceHandler for {}",
                    service_name_for_callback.lock().unwrap()
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
            .unwrap()
            .recv_timeout(std::time::Duration::from_millis(timeout_millis as u64))
            .ok()
    }

    pub fn set_response(&self, time: rosrust::Time, res: T::Response) {
        self.response_sender
            .lock()
            .unwrap()
            .send((time, res))
            .unwrap();
    }
}

#[macro_export(crate)]
macro_rules! define_action_client_with_namespace {
    ($arci_ros_namespace: path, $name: ident,$namespace: path, $action_base:expr) => {
        paste::item! {
            pub struct $name {
                goal_publisher: rosrust::Publisher<$namespace::[<$action_base ActionGoal>]>,
                result_subscriber: $arci_ros_namespace::SubscriberHandler<$namespace::[<$action_base ActionResult>]>,
                cancel_publisher: rosrust::Publisher<msg::actionlib_msgs::GoalID>,
                monitoring_rate: f64,
            }
            impl $name {
                pub fn new(base_topic: &str, queue_size: usize, monitoring_rate: f64) -> Self {
                    let goal_topic = format!("{}/goal", base_topic);
                    let cancel_topic = format!("{}/cancel", base_topic);
                    let goal_publisher =
                        rosrust::publish(&goal_topic, queue_size).unwrap();
                    // TODO: use fifo instead of single buffer in SubscriberHandler
                    let result_subscriber = $arci_ros_namespace::SubscriberHandler::<$namespace::[<$action_base ActionResult>]>::new(
                        &format!("{}/result", base_topic),
                        queue_size,
                    );
                    let cancel_publisher =
                        rosrust::publish(&cancel_topic, queue_size).unwrap();
                        rosrust::ros_info!("Waiting {} ....", goal_topic);
                    $arci_ros_namespace::wait_subscriber(&goal_publisher);
                    rosrust::ros_info!("Waiting {} Done", goal_topic);
                    rosrust::ros_info!("Waiting {} ....", cancel_topic);
                    $arci_ros_namespace::wait_subscriber(&cancel_publisher);
                    rosrust::ros_info!("Waiting {} Done", cancel_topic);
                    Self {
                        goal_publisher,
                        result_subscriber,
                        cancel_publisher,
                        monitoring_rate,
                     }
                }
                #[allow(dead_code)]
                pub fn send_goal_and_wait(&self, goal: $namespace::[<$action_base Goal>], timeout: std::time::Duration) -> Result<$namespace::[<$action_base Result>], $arci_ros_namespace::Error> {
                    let goal_id = self.send_goal(goal)?;
                    self.wait_for_result(&goal_id, timeout)
                }
                #[allow(clippy::field_reassign_with_default)]
                pub fn send_goal(&self, goal: $namespace::[<$action_base Goal>]) -> Result<String, $arci_ros_namespace::Error> {
                    let mut action_goal = <$namespace::[<$action_base ActionGoal>]>::default();
                    action_goal.goal = goal;
                    let goal_id = format!("{}-{}", rosrust::name(), rosrust::now().seconds());
                    action_goal.goal_id.id = goal_id.clone();
                    if self.goal_publisher.send(action_goal).is_err() {
                        return Err($arci_ros_namespace::Error::ActionGoalSendingFailure);
                    }
                    Ok(goal_id)
                }
                pub fn wait_for_result(&self, goal_id: &str, timeout: std::time::Duration) -> Result<$namespace::[<$action_base Result>], $arci_ros_namespace::Error> {
                    let rate = rosrust::rate(self.monitoring_rate);
                    let start_time = std::time::Instant::now();
                    while (start_time.elapsed() < timeout) {
                        if let Some(result) = self.result_subscriber.get()? {
                            if result.status.goal_id.id == *goal_id {
                                // TODO more detailed error / status handling
                                match result.status.status {
                                    msg::actionlib_msgs::GoalStatus::SUCCEEDED => {
                                        return Ok(result.result);
                                    },
                                    msg::actionlib_msgs::GoalStatus::PREEMPTED => {
                                        return Err($arci_ros_namespace::Error::ActionResultPreempted(format!("{:?}", result)));
                                    },
                                    _ => {
                                        return Err($arci_ros_namespace::Error::ActionResultNotSuccess(format!("{:?}", result)));
                                    }
                                }
                            }
                        }
                        rate.sleep();
                    }
                    Err($arci_ros_namespace::Error::ActionResultTimeout)
                }
                #[allow(dead_code)]
                pub fn cancel_goal(&self, goal_id: &str) -> Result<(), $arci_ros_namespace::Error> {
                    if self.cancel_publisher.send(
                    msg::actionlib_msgs::GoalID { id: goal_id.to_owned(), ..Default::default()}).is_err() {
                        return Err($arci_ros_namespace::Error::ActionCancelSendingFailure);
                    }
                    Ok(())
                }
                #[allow(dead_code)]
                pub fn cancel_all_goal(&self) -> Result<(), $arci_ros_namespace::Error> {
                    self.cancel_goal("")
                }
            }
        }
    };
}

#[macro_export(crate)]
macro_rules! define_action_client_internal {
    ($name: ident,$namespace: path, $action_base:expr) => {
        crate::define_action_client_with_namespace!(crate, $name, $namespace, $action_base);
    };
}

#[macro_export]
macro_rules! define_action_client {
    ($name: ident, $namespace: path, $action_base:expr) => {
        ::arci_ros::define_action_client_with_namespace!(
            ::arci_ros,
            $name,
            $namespace,
            $action_base
        );
    };
}
