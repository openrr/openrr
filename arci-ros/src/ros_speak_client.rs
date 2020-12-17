mod msg {
    rosrust::rosmsg_include!(std_msgs / String);
}

pub struct RosEspeakClient {
    publisher: rosrust::Publisher<msg::std_msgs::String>,
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
        let ros_msg = msg::std_msgs::String {
            data: message.to_string(),
        };
        self.publisher.send(ros_msg).unwrap();
    }
}
