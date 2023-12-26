use std::sync::{Arc, Mutex};

use anyhow::format_err;
pub use ros2_client::NodeOptions;
use ros2_client::{action, Name, NodeName, ServiceMapping};
use rustdds::Topic;
use serde::{de::DeserializeOwned, Serialize};

use crate::{msg::MessageType, utils};

/// ROS2 node. This is a wrapper around `Arc<Mutex<ros2_client::Node>>`.
#[derive(Clone)]
pub struct Node {
    pub(crate) inner: Arc<NodeInner>,
}

pub(crate) struct NodeInner {
    pub(crate) node: Mutex<ros2_client::Node>,
}

impl Node {
    /// Creates a new ROS2 node.
    pub fn new(name: &str, namespace: &str, options: NodeOptions) -> Result<Self, arci::Error> {
        let context = ros2_client::Context::new().map_err(anyhow::Error::from)?;
        Self::with_context(&context, name, namespace, options)
    }

    /// Creates a new ROS2 node with `ros2_client::Context`.
    pub fn with_context(
        context: &ros2_client::Context,
        name: &str,
        namespace: &str,
        options: NodeOptions,
    ) -> Result<Self, arci::Error> {
        let name = NodeName::new(namespace, name).map_err(|e| format_err!("{e}"))?;
        let node = context
            .new_node(name, options)
            .map_err(anyhow::Error::from)?;
        Ok(Self {
            inner: Arc::new(NodeInner {
                node: Mutex::new(node),
            }),
        })
    }

    pub fn create_topic<M: MessageType>(&self, name: &str) -> Result<Topic, arci::Error> {
        let name = Name::parse(name).map_err(|e| format_err!("{e}"))?;
        self.inner
            .node
            .lock()
            .unwrap()
            .create_topic(&name, M::message_type_name(), &utils::topic_qos())
            .map_err(|e| arci::Error::Other(e.into()))
    }

    pub fn create_publisher<M: Serialize>(
        &self,
        topic: &Topic,
    ) -> Result<ros2_client::Publisher<M>, arci::Error> {
        self.inner
            .node
            .lock()
            .unwrap()
            .create_publisher(topic, None)
            .map_err(|e| arci::Error::Other(e.into()))
    }

    pub fn create_subscription<M: DeserializeOwned + 'static>(
        &self,
        topic: &Topic,
    ) -> Result<ros2_client::Subscription<M>, arci::Error> {
        self.inner
            .node
            .lock()
            .unwrap()
            .create_subscription(topic, None)
            .map_err(|e| arci::Error::Other(e.into()))
    }

    pub fn create_client<S: ros2_client::Service + MessageType + 'static>(
        &self,
        name: &str,
    ) -> Result<ros2_client::Client<S>, arci::Error>
    where
        S::Request: Clone,
    {
        let name = Name::parse(name).map_err(|e| format_err!("{e}"))?;
        self.inner
            .node
            .lock()
            .unwrap()
            .create_client(
                ServiceMapping::Enhanced,
                &name,
                &S::service_type_name(),
                utils::service_qos(),
                utils::service_qos(),
            )
            .map_err(|e| arci::Error::Other(e.into()))
    }

    pub fn create_server<S: ros2_client::Service + MessageType + 'static>(
        &self,
        name: &str,
    ) -> Result<ros2_client::Server<S>, arci::Error>
    where
        S::Request: Clone,
    {
        let name = Name::parse(name).map_err(|e| format_err!("{e}"))?;
        self.inner
            .node
            .lock()
            .unwrap()
            .create_server(
                ServiceMapping::Enhanced,
                &name,
                &S::service_type_name(),
                utils::service_qos(),
                utils::service_qos(),
            )
            .map_err(|e| arci::Error::Other(e.into()))
    }

    pub fn create_action_client<A: ros2_client::ActionTypes + MessageType + 'static>(
        &self,
        action_name: &str,
    ) -> Result<action::ActionClient<A>, arci::Error> {
        let action_name = Name::parse(action_name).map_err(|e| format_err!("{e}"))?;
        self.inner
            .node
            .lock()
            .unwrap()
            .create_action_client(
                ServiceMapping::Enhanced,
                &action_name,
                &A::action_type_name(),
                utils::action_client_qos(),
            )
            .map_err(|e| arci::Error::Other(e.into()))
    }

    pub fn create_action_server<A: ros2_client::ActionTypes + MessageType + 'static>(
        &self,
        action_name: &str,
    ) -> Result<action::ActionServer<A>, arci::Error> {
        let action_name = Name::parse(action_name).map_err(|e| format_err!("{e}"))?;
        self.inner
            .node
            .lock()
            .unwrap()
            .create_action_server(
                ServiceMapping::Enhanced,
                &action_name,
                &A::action_type_name(),
                utils::action_server_qos(),
            )
            .map_err(|e| arci::Error::Other(e.into()))
    }

    pub fn spinner(&self) -> ros2_client::Spinner {
        self.inner.node.lock().unwrap().spinner()
    }
}
