use async_trait::async_trait;
use serde::{Deserialize, Serialize};

mod msg {
    rosrust::rosmsg_include!(std_msgs / String);
}

pub struct RosEspeakClient {
    publisher: rosrust::Publisher<msg::std_msgs::String>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
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

#[async_trait]
impl arci::Speaker for RosEspeakClient {
    async fn speak(&self, message: &str) -> Result<(), arci::Error> {
        let ros_msg = msg::std_msgs::String {
            data: message.to_string(),
        };
        self.publisher
            .send(ros_msg)
            // TODO: use anyhow::Error::from once rosrust::Error implements Sync.
            .map_err(|e| anyhow::format_err!("{}", e))?;
        Ok(())
    }
}
