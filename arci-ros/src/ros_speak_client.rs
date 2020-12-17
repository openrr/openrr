mod msg {
    rosrust::rosmsg_include!(espeak_server_ros / Message,);
}

pub struct RosEspeakClient {
    publisher: rosrust::Publisher<msg::espeak_server_ros::Message>,
}

impl RosEspeakClient {
    pub fn new(topic: &str) -> Self {
        Self {
            publisher: rosrust::publish(topic, 2).unwrap(),
        }
    }
}

impl arci::Speaker for RosEspeakClient {
    fn speak(&self, message: &str) {
        let mut mode = msg::espeak_server_ros::Message::default();
        mode.message = message.into();
        self.publisher.send(mode).unwrap();
    }
}
