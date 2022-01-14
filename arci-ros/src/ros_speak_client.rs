use arci::WaitFuture;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

mod msg {
    rosrust::rosmsg_include!(std_msgs / String);
}

pub struct RosEspeakClient {
    publisher: rosrust::Publisher<msg::std_msgs::String>,
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RosEspeakClientConfig {
    pub topic: String,
}

impl RosEspeakClient {
    pub fn new(topic: &str) -> Self {
        Self {
            publisher: rosrust::publish(topic, 2).unwrap(),
        }
    }
}

impl arci::Speaker for RosEspeakClient {
    fn speak(&self, message: &str) -> Result<WaitFuture, arci::Error> {
        let ros_msg = msg::std_msgs::String {
            data: message.to_string(),
        };
        self.publisher
            .send(ros_msg)
            .map_err(|e| anyhow::format_err!("{e}"))?;
        Ok(WaitFuture::ready())
    }
}
