#![doc = include_str!("../README.md")]

mod pb {
    #![allow(unreachable_pub)]

    #[cfg(local_out_dir)]
    include!("generated/arci.rs");
    #[cfg(not(local_out_dir))]
    tonic::include_proto!("arci");
}

#[rustfmt::skip]
#[path = "gen/impls.rs"]
mod impls;

use std::{
    future::Future,
    net::SocketAddr,
    time::{Duration, SystemTime},
};

use arci::nalgebra;
use tracing::error;

pub use crate::impls::*;

fn block_in_place<T>(f: impl Future<Output = T>) -> T {
    tokio::task::block_in_place(|| tokio::runtime::Handle::current().block_on(f))
}

fn wait_from_handle(
    handle: tokio::task::JoinHandle<Result<tonic::Response<()>, tonic::Status>>,
) -> arci::WaitFuture {
    arci::WaitFuture::new(async move {
        handle
            .await
            .map_err(|e| arci::Error::Other(e.into()))?
            .map_err(|e| arci::Error::Other(e.into()))?;
        Ok(())
    })
}

// =============================================================================
// arci::JointTrajectoryClient

impl arci::JointTrajectoryClient for RemoteJointTrajectoryClientSender {
    fn joint_names(&self) -> Vec<String> {
        let mut client = self.client.clone();
        block_in_place(client.joint_names(()))
            .unwrap()
            .into_inner()
            .names
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        let mut client = self.client.clone();
        Ok(block_in_place(client.current_joint_positions(()))
            .map_err(|e| arci::Error::Other(e.into()))?
            .into_inner()
            .positions)
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<arci::WaitFuture, arci::Error> {
        let mut client = self.client.clone();
        Ok(wait_from_handle(tokio::spawn(async move {
            client
                .send_joint_positions(pb::JointPositionsRequest {
                    positions,
                    duration: Some(duration.try_into().unwrap()),
                })
                .await
        })))
    }

    fn send_joint_trajectory(
        &self,
        trajectory: Vec<arci::TrajectoryPoint>,
    ) -> Result<arci::WaitFuture, arci::Error> {
        let mut client = self.client.clone();
        Ok(wait_from_handle(tokio::spawn(async move {
            client
                .send_joint_trajectory(pb::JointTrajectoryRequest {
                    trajectory: trajectory.into_iter().map(Into::into).collect(),
                })
                .await
        })))
    }
}

#[tonic::async_trait]
impl<C> pb::joint_trajectory_client_server::JointTrajectoryClient
    for RemoteJointTrajectoryClientReceiver<C>
where
    C: arci::JointTrajectoryClient + 'static,
{
    async fn joint_names(
        &self,
        _: tonic::Request<()>,
    ) -> Result<tonic::Response<pb::JointNamesResponse>, tonic::Status> {
        Ok(tonic::Response::new(pb::JointNamesResponse {
            names: arci::JointTrajectoryClient::joint_names(&self.inner),
        }))
    }

    async fn current_joint_positions(
        &self,
        _: tonic::Request<()>,
    ) -> Result<tonic::Response<pb::JointPositionsResponse>, tonic::Status> {
        Ok(tonic::Response::new(pb::JointPositionsResponse {
            positions: arci::JointTrajectoryClient::current_joint_positions(&self.inner)
                .map_err(|e| tonic::Status::unknown(e.to_string()))?,
        }))
    }

    async fn send_joint_positions(
        &self,
        request: tonic::Request<pb::JointPositionsRequest>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let request = request.into_inner();
        arci::JointTrajectoryClient::send_joint_positions(
            &self.inner,
            request.positions,
            request.duration.unwrap().try_into().unwrap(),
        )
        .map_err(|e| tonic::Status::unknown(e.to_string()))?
        .await
        .map_err(|e| tonic::Status::unknown(e.to_string()))?;
        Ok(tonic::Response::new(()))
    }

    async fn send_joint_trajectory(
        &self,
        request: tonic::Request<pb::JointTrajectoryRequest>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let request = request.into_inner();
        arci::JointTrajectoryClient::send_joint_trajectory(
            &self.inner,
            request.trajectory.into_iter().map(Into::into).collect(),
        )
        .map_err(|e| tonic::Status::unknown(e.to_string()))?
        .await
        .map_err(|e| tonic::Status::unknown(e.to_string()))?;
        Ok(tonic::Response::new(()))
    }
}

// =============================================================================
// arci::Gamepad

#[arci::async_trait]
impl arci::Gamepad for RemoteGamepadSender {
    async fn next_event(&self) -> arci::gamepad::GamepadEvent {
        let mut client = self.client.clone();
        match client.next_event(()).await {
            Ok(event) => event.into_inner().into(),
            Err(e) => {
                error!("{e}");
                arci::gamepad::GamepadEvent::Unknown
            }
        }
    }

    fn stop(&self) {
        let mut client = self.client.clone();
        if let Err(e) = block_in_place(client.stop(())) {
            error!("{e}");
        }
    }
}

#[tonic::async_trait]
impl<C> pb::gamepad_server::Gamepad for RemoteGamepadReceiver<C>
where
    C: arci::Gamepad + 'static,
{
    async fn next_event(
        &self,
        _: tonic::Request<()>,
    ) -> Result<tonic::Response<pb::GamepadEvent>, tonic::Status> {
        Ok(tonic::Response::new(self.inner.next_event().await.into()))
    }

    async fn stop(&self, _: tonic::Request<()>) -> Result<tonic::Response<()>, tonic::Status> {
        self.inner.stop();
        Ok(tonic::Response::new(()))
    }
}

// =============================================================================
// Messages

impl From<arci::Isometry2<f64>> for pb::Isometry2 {
    fn from(val: arci::Isometry2<f64>) -> Self {
        Self {
            rotation: Some(pb::UnitComplex {
                re: val.rotation.re,
                im: val.rotation.im,
            }),
            translation: Some(pb::Translation2 {
                x: val.translation.x,
                y: val.translation.y,
            }),
        }
    }
}

impl From<pb::Isometry2> for arci::Isometry2<f64> {
    fn from(val: pb::Isometry2) -> Self {
        let translation = val.translation.unwrap();
        let rotation = val.rotation.unwrap();
        Self::from_parts(
            nalgebra::Translation2::new(translation.x, translation.y),
            nalgebra::UnitComplex::from_complex(nalgebra::Complex {
                re: rotation.re,
                im: rotation.im,
            }),
        )
    }
}

impl From<arci::Isometry3<f64>> for pb::Isometry3 {
    fn from(val: arci::Isometry3<f64>) -> Self {
        Self {
            rotation: Some(pb::UnitQuaternion {
                x: val.rotation.coords.x,
                y: val.rotation.coords.y,
                z: val.rotation.coords.z,
                w: val.rotation.coords.w,
            }),
            translation: Some(pb::Translation3 {
                x: val.translation.x,
                y: val.translation.y,
                z: val.translation.z,
            }),
        }
    }
}

impl From<pb::Isometry3> for arci::Isometry3<f64> {
    fn from(val: pb::Isometry3) -> Self {
        let translation = val.translation.unwrap();
        let rotation = val.rotation.unwrap();
        Self::from_parts(
            nalgebra::Translation3::new(translation.x, translation.y, translation.z),
            nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                rotation.w, rotation.x, rotation.y, rotation.z,
            )),
        )
    }
}

impl From<(arci::Isometry2<f64>, &str, Duration)> for pb::GoalPoseRequest {
    fn from((goal, frame_id, timeout): (arci::Isometry2<f64>, &str, Duration)) -> Self {
        Self {
            goal: Some(goal.into()),
            frame_id: frame_id.into(),
            timeout: Some(timeout.try_into().unwrap()),
        }
    }
}

impl From<(&str, &str, SystemTime)> for pb::ResolveTransformationRequest {
    fn from((from, to, time): (&str, &str, SystemTime)) -> Self {
        Self {
            from: from.into(),
            to: to.into(),
            time: Some(time.into()),
        }
    }
}

impl From<arci::gamepad::GamepadEvent> for pb::GamepadEvent {
    fn from(val: arci::gamepad::GamepadEvent) -> Self {
        let event = match val {
            arci::gamepad::GamepadEvent::ButtonPressed(b) => {
                pb::gamepad_event::Event::ButtonPressed(pb::Button::from(b) as _)
            }
            arci::gamepad::GamepadEvent::ButtonReleased(b) => {
                pb::gamepad_event::Event::ButtonReleased(pb::Button::from(b) as _)
            }
            arci::gamepad::GamepadEvent::AxisChanged(axis, value) => {
                pb::gamepad_event::Event::AxisChanged(pb::AxisChanged {
                    axis: pb::Axis::from(axis) as _,
                    value,
                })
            }
            arci::gamepad::GamepadEvent::Connected => pb::gamepad_event::Event::Connected(()),
            arci::gamepad::GamepadEvent::Disconnected => pb::gamepad_event::Event::Disconnected(()),
            arci::gamepad::GamepadEvent::Unknown => pb::gamepad_event::Event::Unknown(()),
        };
        Self { event: Some(event) }
    }
}

impl From<pb::GamepadEvent> for arci::gamepad::GamepadEvent {
    fn from(val: pb::GamepadEvent) -> Self {
        let val = val.event.unwrap();
        match val {
            pb::gamepad_event::Event::ButtonPressed(b) => {
                Self::ButtonPressed(pb::Button::try_from(b).unwrap().into())
            }
            pb::gamepad_event::Event::ButtonReleased(b) => {
                Self::ButtonReleased(pb::Button::try_from(b).unwrap().into())
            }
            pb::gamepad_event::Event::AxisChanged(a) => {
                Self::AxisChanged(pb::Axis::try_from(a.axis).unwrap().into(), a.value)
            }
            pb::gamepad_event::Event::Connected(()) => Self::Connected,
            pb::gamepad_event::Event::Disconnected(()) => Self::Disconnected,
            pb::gamepad_event::Event::Unknown(()) => Self::Unknown,
        }
    }
}
