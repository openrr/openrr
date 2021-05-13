//! This module defines FFI-safe equivalents of the various types and traits
//! used in arci and openrr-plugin. The types defined by this module will never
//! appear in the public API, and the conversion is done internally.

use std::{
    convert::{TryFrom, TryInto},
    future::Future,
    sync::Arc,
    time::SystemTime,
};

use abi_stable::{
    declare_root_module_statics,
    erased_types::TU_Opaque,
    library::RootModule,
    package_version_strings,
    prefix_type::PrefixTypeTrait,
    rtry, sabi_trait,
    sabi_types::VersionStrings,
    std_types::{RBox, RBoxError, RDuration, ROk, ROption, RStr, RString, RVec},
    StableAbi,
};
use anyhow::format_err;
use num_traits::Float;

use crate::{
    GamepadProxy, JointTrajectoryClientProxy, LocalizationProxy, MoveBaseProxy, NavigationProxy,
    Plugin, PluginProxy, SpeakerProxy, StaticJointTrajectoryClient, TransformResolverProxy,
};

type RResult<T, E = RError> = abi_stable::std_types::RResult<T, E>;

fn block_in_place<T>(f: impl Future<Output = T>) -> T {
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(f)
}

// =============================================================================
// f64

/// FFI-safe equivalent of [`f64`].
///
/// `f64` does not implement `StableAbi`, so convert it to integers by `integer_decode`,
/// and recover it on conversion to `f64`.
///
/// Refs: <https://docs.rs/num-traits/0.2/num_traits/float/trait.Float.html#tymethod.integer_decode>
#[repr(C)]
#[derive(Clone, Copy, StableAbi)]
pub(crate) struct RF64 {
    mantissa: u64,
    exponent: i16,
    sign: i8,
}

impl From<f64> for RF64 {
    fn from(val: f64) -> Self {
        let (mantissa, exponent, sign) = Float::integer_decode(val);
        Self {
            mantissa,
            exponent,
            sign,
        }
    }
}

impl From<RF64> for f64 {
    fn from(val: RF64) -> Self {
        let RF64 {
            mantissa,
            exponent,
            sign,
        } = val;
        let sign_f = sign as f64;
        let mantissa_f = mantissa as f64;
        let exponent_f = 2.0.powf(exponent as f64);
        sign_f * mantissa_f * exponent_f
    }
}

// =============================================================================
// std::time::SystemTime

/// FFI-safe equivalent of [`std::time::SystemTime`].
///
/// `SystemTime` does not implement `StableAbi`, so convert it to duration since unix epoch,
/// and recover it on conversion to `SystemTime`.
/// This is inspired by the way `serde` implements `Serialize`/`Deserialize` on `SystemTime`.
///
/// Refs:
/// - <https://github.com/serde-rs/serde/blob/v1.0.126/serde/src/ser/impls.rs#L610-L625>
/// - <https://github.com/serde-rs/serde/blob/v1.0.126/serde/src/de/impls.rs#L1993-L2138>
#[repr(C)]
#[derive(StableAbi)]
pub(crate) struct RSystemTime {
    duration_since_epoch: RDuration,
}

impl TryFrom<SystemTime> for RSystemTime {
    type Error = RError;

    fn try_from(val: SystemTime) -> Result<Self, Self::Error> {
        let duration_since_epoch = val
            .duration_since(SystemTime::UNIX_EPOCH)
            .map_err(|_| format_err!("SystemTime must be later than UNIX_EPOCH"))?;
        Ok(Self {
            duration_since_epoch: duration_since_epoch.into(),
        })
    }
}

impl TryFrom<RSystemTime> for SystemTime {
    type Error = RError;

    fn try_from(val: RSystemTime) -> Result<Self, Self::Error> {
        let duration_since_epoch = val.duration_since_epoch.into();
        SystemTime::UNIX_EPOCH
            .checked_add(duration_since_epoch)
            .ok_or_else(|| format_err!("overflow deserializing SystemTime").into())
    }
}

// =============================================================================
// nalgebra::Isometry2<f64>

/// FFI-safe equivalent of [`nalgebra::Isometry2<f64>`](nalgebra::Isometry2).
#[repr(C)]
#[derive(StableAbi)]
pub(crate) struct RIsometry2F64 {
    rotation: RUnitComplexF64,
    translation: RTranslation2F64,
}

impl From<nalgebra::Isometry2<f64>> for RIsometry2F64 {
    fn from(val: nalgebra::Isometry2<f64>) -> Self {
        Self {
            rotation: val.rotation.into(),
            translation: val.translation.into(),
        }
    }
}

impl From<RIsometry2F64> for nalgebra::Isometry2<f64> {
    fn from(val: RIsometry2F64) -> Self {
        Self::from_parts(val.translation.into(), val.rotation.into())
    }
}

/// FFI-safe equivalent of [`nalgebra::UnitComplex<f64>`](nalgebra::UnitComplex).
#[repr(C)]
#[derive(StableAbi)]
struct RUnitComplexF64 {
    re: RF64,
    im: RF64,
}

impl From<nalgebra::UnitComplex<f64>> for RUnitComplexF64 {
    fn from(val: nalgebra::UnitComplex<f64>) -> Self {
        let val = val.into_inner();
        Self {
            re: val.re.into(),
            im: val.im.into(),
        }
    }
}

impl From<RUnitComplexF64> for nalgebra::UnitComplex<f64> {
    fn from(val: RUnitComplexF64) -> Self {
        Self::from_complex(nalgebra::Complex {
            re: val.re.into(),
            im: val.im.into(),
        })
    }
}

/// FFI-safe equivalent of [`nalgebra::Translation2<f64>`](nalgebra::Translation2).
#[repr(C)]
#[derive(StableAbi)]
struct RTranslation2F64 {
    x: RF64,
    y: RF64,
}

impl From<nalgebra::Translation2<f64>> for RTranslation2F64 {
    fn from(val: nalgebra::Translation2<f64>) -> Self {
        Self {
            x: val.vector.x.into(),
            y: val.vector.y.into(),
        }
    }
}

impl From<RTranslation2F64> for nalgebra::Translation2<f64> {
    fn from(val: RTranslation2F64) -> Self {
        Self::new(val.x.into(), val.y.into())
    }
}

// =============================================================================
// nalgebra::Isometry3<f64>

/// FFI-safe equivalent of [`nalgebra::Isometry3<f64>`](nalgebra::Isometry3).
#[repr(C)]
#[derive(StableAbi)]
pub(crate) struct RIsometry3F64 {
    rotation: RUnitQuaternionF64,
    translation: RTranslation3F64,
}

impl From<nalgebra::Isometry3<f64>> for RIsometry3F64 {
    fn from(val: nalgebra::Isometry3<f64>) -> Self {
        Self {
            rotation: val.rotation.into(),
            translation: val.translation.into(),
        }
    }
}

impl From<RIsometry3F64> for nalgebra::Isometry3<f64> {
    fn from(val: RIsometry3F64) -> Self {
        Self::from_parts(val.translation.into(), val.rotation.into())
    }
}

/// FFI-safe equivalent of [`nalgebra::UnitQuaternion<f64>`](nalgebra::UnitQuaternion).
#[repr(C)]
#[derive(StableAbi)]
struct RUnitQuaternionF64 {
    x: RF64,
    y: RF64,
    z: RF64,
    w: RF64,
}

impl From<nalgebra::UnitQuaternion<f64>> for RUnitQuaternionF64 {
    fn from(val: nalgebra::UnitQuaternion<f64>) -> Self {
        let val = val.into_inner();
        Self {
            x: val.coords.x.into(),
            y: val.coords.y.into(),
            z: val.coords.z.into(),
            w: val.coords.w.into(),
        }
    }
}

impl From<RUnitQuaternionF64> for nalgebra::UnitQuaternion<f64> {
    fn from(val: RUnitQuaternionF64) -> Self {
        Self::from_quaternion(nalgebra::Quaternion::new(
            val.w.into(),
            val.x.into(),
            val.y.into(),
            val.z.into(),
        ))
    }
}

/// FFI-safe equivalent of [`nalgebra::Translation3<f64>`](nalgebra::Translation3).
#[repr(C)]
#[derive(StableAbi)]
struct RTranslation3F64 {
    x: RF64,
    y: RF64,
    z: RF64,
}

impl From<nalgebra::Translation3<f64>> for RTranslation3F64 {
    fn from(val: nalgebra::Translation3<f64>) -> Self {
        Self {
            x: val.vector.x.into(),
            y: val.vector.y.into(),
            z: val.vector.z.into(),
        }
    }
}

impl From<RTranslation3F64> for nalgebra::Translation3<f64> {
    fn from(val: RTranslation3F64) -> Self {
        Self::new(val.x.into(), val.y.into(), val.z.into())
    }
}

// =============================================================================
// arci::Error

/// FFI-safe equivalent of [`arci::Error`].
#[repr(C)]
#[derive(StableAbi)]
pub(crate) struct RError {
    repr: RBoxError,
}

impl From<arci::Error> for RError {
    fn from(e: arci::Error) -> Self {
        Self {
            // TODO: propagate error kind.
            repr: RBoxError::from_box(e.into()),
        }
    }
}

impl From<anyhow::Error> for RError {
    fn from(e: anyhow::Error) -> Self {
        Self {
            // TODO: propagate error kind.
            repr: RBoxError::from_box(e.into()),
        }
    }
}

impl From<RError> for arci::Error {
    fn from(e: RError) -> Self {
        // TODO: propagate error kind.
        Self::Other(format_err!("{}", e.repr))
    }
}

// =============================================================================
// arci::WaitFuture<'static>

#[repr(C)]
#[derive(StableAbi)]
#[must_use]
pub(crate) struct RBlockingWait(RBoxWait);

impl RBlockingWait {
    fn from_fn(f: impl FnOnce() -> RResult<()> + Send + 'static) -> Self {
        Self(RBoxWait::from_value(f, TU_Opaque))
    }
}

impl From<arci::WaitFuture<'static>> for RBlockingWait {
    fn from(wait: arci::WaitFuture<'static>) -> Self {
        Self::from_fn(move || block_in_place(wait).map_err(RError::from).into())
    }
}

impl From<RBlockingWait> for arci::WaitFuture<'static> {
    fn from(wait: RBlockingWait) -> Self {
        // Creates a WaitFuture that waits until Wait::wait done only if the future
        // is polled. This future is a bit tricky, but it's more efficient than
        // using only `tokio::task::spawn_blocking` because it doesn't spawn a thread
        // if the WaitFuture is ignored.
        arci::WaitFuture::new(async move {
            tokio::task::spawn_blocking(move || wait.0.wait())
                .await
                .map_err(|e| arci::Error::Other(e.into()))?
                .into_result()?;
            Ok(())
        })
    }
}

type RBoxWait = RWaitTrait_TO<RBox<()>>;

#[sabi_trait]
trait RWaitTrait: Send + 'static {
    fn wait(self) -> RResult<()>;
}

impl<F> RWaitTrait for F
where
    F: FnOnce() -> RResult<()> + Send + 'static,
{
    fn wait(self) -> RResult<()> {
        self()
    }
}

// =============================================================================
// arci::JointTrajectoryClient

/// FFI-safe equivalent of [`Box<dyn arci::JointTrajectoryClient>`](arci::JointTrajectoryClient).
pub(crate) type JointTrajectoryClientTraitObject = RJointTrajectoryClientTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RJointTrajectoryClientTrait: Send + Sync + 'static {
    fn joint_names(&self) -> RVec<RString>;
    fn current_joint_positions(&self) -> RResult<RVec<RF64>>;
    fn send_joint_positions(
        &self,
        positions: RVec<RF64>,
        duration: RDuration,
    ) -> RResult<RBlockingWait>;
    fn send_joint_trajectory(&self, trajectory: RVec<RTrajectoryPoint>) -> RResult<RBlockingWait>;
}

impl<T> RJointTrajectoryClientTrait for T
where
    T: ?Sized + StaticJointTrajectoryClient,
{
    fn joint_names(&self) -> RVec<RString> {
        StaticJointTrajectoryClient::joint_names(self)
            .into_iter()
            .map(|s| s.into())
            .collect()
    }

    fn current_joint_positions(&self) -> RResult<RVec<RF64>> {
        ROk(
            rtry!(StaticJointTrajectoryClient::current_joint_positions(self))
                .into_iter()
                .map(RF64::from)
                .collect(),
        )
    }

    fn send_joint_positions(
        &self,
        positions: RVec<RF64>,
        duration: RDuration,
    ) -> RResult<RBlockingWait> {
        ROk(rtry!(StaticJointTrajectoryClient::send_joint_positions(
            self,
            positions.into_iter().map(f64::from).collect(),
            duration.into(),
        ))
        .into())
    }

    fn send_joint_trajectory(&self, trajectory: RVec<RTrajectoryPoint>) -> RResult<RBlockingWait> {
        ROk(rtry!(StaticJointTrajectoryClient::send_joint_trajectory(
            self,
            trajectory
                .into_iter()
                .map(arci::TrajectoryPoint::from)
                .collect(),
        ))
        .into())
    }
}

/// FFI-safe equivalent of [`arci::TrajectoryPoint`].
#[repr(C)]
#[derive(StableAbi)]
pub(crate) struct RTrajectoryPoint {
    positions: RVec<RF64>,
    velocities: ROption<RVec<RF64>>,
    time_from_start: RDuration,
}

impl From<arci::TrajectoryPoint> for RTrajectoryPoint {
    fn from(val: arci::TrajectoryPoint) -> Self {
        Self {
            positions: val.positions.into_iter().map(RF64::from).collect(),
            velocities: val
                .velocities
                .map(|v| v.into_iter().map(RF64::from).collect())
                .into(),
            time_from_start: val.time_from_start.into(),
        }
    }
}

impl From<RTrajectoryPoint> for arci::TrajectoryPoint {
    fn from(val: RTrajectoryPoint) -> Self {
        Self {
            positions: val.positions.into_iter().map(f64::from).collect(),
            velocities: val
                .velocities
                .into_option()
                .map(|v| v.into_iter().map(f64::from).collect()),
            time_from_start: val.time_from_start.into(),
        }
    }
}

// =============================================================================
// arci::Speaker

/// FFI-safe equivalent of [`Box<dyn arci::Speaker>`](arci::Speaker).
pub(crate) type SpeakerTraitObject = RSpeakerTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RSpeakerTrait: Send + Sync + 'static {
    fn speak(&self, message: RStr<'_>) -> RResult<RBlockingWait>;
}

impl<T> RSpeakerTrait for T
where
    T: ?Sized + arci::Speaker + 'static,
{
    fn speak(&self, message: RStr<'_>) -> RResult<RBlockingWait> {
        ROk(rtry!(arci::Speaker::speak(self, message.into())).into())
    }
}

// =============================================================================
// arci::MoveBase

/// FFI-safe equivalent of [`Box<dyn arci::MoveBase>`](arci::MoveBase).
pub(crate) type MoveBaseTraitObject = RMoveBaseTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RMoveBaseTrait: Send + Sync + 'static {
    fn send_velocity(&self, velocity: RBaseVelocity) -> RResult<()>;
    fn current_velocity(&self) -> RResult<RBaseVelocity>;
}

impl<T> RMoveBaseTrait for T
where
    T: ?Sized + arci::MoveBase + 'static,
{
    fn send_velocity(&self, velocity: RBaseVelocity) -> RResult<()> {
        rtry!(arci::MoveBase::send_velocity(
            self,
            &arci::BaseVelocity::from(velocity)
        ));
        ROk(())
    }

    fn current_velocity(&self) -> RResult<RBaseVelocity> {
        ROk(rtry!(arci::MoveBase::current_velocity(self)).into())
    }
}

/// FFI-safe equivalent of [`arci::BaseVelocity`].
#[repr(C)]
#[derive(StableAbi)]
pub(crate) struct RBaseVelocity {
    x: RF64,
    y: RF64,
    theta: RF64,
}

impl From<arci::BaseVelocity> for RBaseVelocity {
    fn from(val: arci::BaseVelocity) -> Self {
        Self {
            x: val.x.into(),
            y: val.y.into(),
            theta: val.theta.into(),
        }
    }
}

impl From<RBaseVelocity> for arci::BaseVelocity {
    fn from(val: RBaseVelocity) -> Self {
        Self {
            x: val.x.into(),
            y: val.y.into(),
            theta: val.theta.into(),
        }
    }
}

// =============================================================================
// arci::Navigation

/// FFI-safe equivalent of [`Box<dyn arci::Navigation>`](arci::Navigation).
pub(crate) type NavigationTraitObject = RNavigationTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RNavigationTrait: Send + Sync + 'static {
    fn send_goal_pose(
        &self,
        goal: RIsometry2F64,
        frame_id: RStr<'_>,
        timeout: RDuration,
    ) -> RResult<RBlockingWait>;

    fn cancel(&self) -> RResult<()>;
}

impl<T> RNavigationTrait for T
where
    T: ?Sized + arci::Navigation + 'static,
{
    fn send_goal_pose(
        &self,
        goal: RIsometry2F64,
        frame_id: RStr<'_>,
        timeout: RDuration,
    ) -> RResult<RBlockingWait> {
        ROk(rtry!(arci::Navigation::send_goal_pose(
            self,
            goal.into(),
            frame_id.into(),
            timeout.into(),
        ))
        .into())
    }

    fn cancel(&self) -> RResult<()> {
        rtry!(arci::Navigation::cancel(self));
        ROk(())
    }
}

// =============================================================================
// arci::Localization

/// FFI-safe equivalent of [`Box<dyn arci::Localization>`](arci::Localization).
pub(crate) type LocalizationTraitObject = RLocalizationTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RLocalizationTrait: Send + Sync + 'static {
    fn current_pose(&self, frame_id: RStr<'_>) -> RResult<RIsometry2F64>;
}

impl<T> RLocalizationTrait for T
where
    T: arci::Localization + 'static,
{
    fn current_pose(&self, frame_id: RStr<'_>) -> RResult<RIsometry2F64> {
        ROk(rtry!(arci::Localization::current_pose(self, frame_id.into())).into())
    }
}

// =============================================================================
// arci::TransformResolver

/// FFI-safe equivalent of [`Box<dyn arci::TransformResolver>`](arci::TransformResolver).
pub(crate) type TransformResolverTraitObject = RTransformResolverTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RTransformResolverTrait: Send + Sync + 'static {
    fn resolve_transformation(
        &self,
        from: RStr<'_>,
        to: RStr<'_>,
        time: RSystemTime,
    ) -> RResult<RIsometry3F64>;
}

impl<T> RTransformResolverTrait for T
where
    T: ?Sized + arci::TransformResolver + 'static,
{
    fn resolve_transformation(
        &self,
        from: RStr<'_>,
        to: RStr<'_>,
        time: RSystemTime,
    ) -> RResult<RIsometry3F64> {
        ROk(rtry!(arci::TransformResolver::resolve_transformation(
            self,
            from.into(),
            to.into(),
            rtry!(time.try_into()),
        ))
        .into())
    }
}

// =============================================================================
// arci::Gamepad

/// FFI-safe equivalent of [`Arc<dyn arci::Gamepad>`](arci::Gamepad).
pub(crate) type GamepadTraitObject = RGamepadTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RGamepadTrait: Send + Sync + Clone + 'static {
    fn next_event(&self) -> RGamepadEvent;
    fn stop(&self);
}

impl<T> RGamepadTrait for Arc<T>
where
    T: ?Sized + arci::Gamepad + 'static,
{
    fn next_event(&self) -> RGamepadEvent {
        block_in_place(arci::Gamepad::next_event(&**self)).into()
    }

    fn stop(&self) {
        arci::Gamepad::stop(&**self);
    }
}

#[repr(C)]
#[derive(StableAbi)]
pub(crate) enum RButton {
    South,
    East,
    North,
    West,
    LeftTrigger,
    LeftTrigger2,
    RightTrigger,
    RightTrigger2,
    Select,
    Start,
    Mode,
    LeftThumb,
    RightThumb,
    DPadUp,
    DPadDown,
    DPadLeft,
    DPadRight,
    Unknown,
}

impl From<arci::gamepad::Button> for RButton {
    fn from(b: arci::gamepad::Button) -> Self {
        match b {
            arci::gamepad::Button::South => Self::South,
            arci::gamepad::Button::East => Self::East,
            arci::gamepad::Button::North => Self::North,
            arci::gamepad::Button::West => Self::West,
            arci::gamepad::Button::LeftTrigger => Self::LeftTrigger,
            arci::gamepad::Button::LeftTrigger2 => Self::LeftTrigger2,
            arci::gamepad::Button::RightTrigger => Self::RightTrigger,
            arci::gamepad::Button::RightTrigger2 => Self::RightTrigger2,
            arci::gamepad::Button::Select => Self::Select,
            arci::gamepad::Button::Start => Self::Start,
            arci::gamepad::Button::Mode => Self::Mode,
            arci::gamepad::Button::LeftThumb => Self::LeftThumb,
            arci::gamepad::Button::RightThumb => Self::RightThumb,
            arci::gamepad::Button::DPadUp => Self::DPadUp,
            arci::gamepad::Button::DPadDown => Self::DPadDown,
            arci::gamepad::Button::DPadLeft => Self::DPadLeft,
            arci::gamepad::Button::DPadRight => Self::DPadRight,
            arci::gamepad::Button::Unknown => Self::Unknown,
        }
    }
}

impl From<RButton> for arci::gamepad::Button {
    fn from(b: RButton) -> Self {
        match b {
            RButton::South => Self::South,
            RButton::East => Self::East,
            RButton::North => Self::North,
            RButton::West => Self::West,
            RButton::LeftTrigger => Self::LeftTrigger,
            RButton::LeftTrigger2 => Self::LeftTrigger2,
            RButton::RightTrigger => Self::RightTrigger,
            RButton::RightTrigger2 => Self::RightTrigger2,
            RButton::Select => Self::Select,
            RButton::Start => Self::Start,
            RButton::Mode => Self::Mode,
            RButton::LeftThumb => Self::LeftThumb,
            RButton::RightThumb => Self::RightThumb,
            RButton::DPadUp => Self::DPadUp,
            RButton::DPadDown => Self::DPadDown,
            RButton::DPadLeft => Self::DPadLeft,
            RButton::DPadRight => Self::DPadRight,
            RButton::Unknown => Self::Unknown,
        }
    }
}

#[repr(C)]
#[derive(StableAbi)]
pub(crate) enum RAxis {
    LeftStickX,
    LeftStickY,
    LeftTrigger,
    RightStickX,
    RightStickY,
    RightTrigger,
    DPadX,
    DPadY,
    Unknown,
}

impl From<arci::gamepad::Axis> for RAxis {
    fn from(a: arci::gamepad::Axis) -> Self {
        match a {
            arci::gamepad::Axis::LeftStickX => Self::LeftStickX,
            arci::gamepad::Axis::LeftStickY => Self::LeftStickY,
            arci::gamepad::Axis::LeftTrigger => Self::LeftTrigger,
            arci::gamepad::Axis::RightStickX => Self::RightStickX,
            arci::gamepad::Axis::RightStickY => Self::RightStickY,
            arci::gamepad::Axis::RightTrigger => Self::RightTrigger,
            arci::gamepad::Axis::DPadX => Self::DPadX,
            arci::gamepad::Axis::DPadY => Self::DPadY,
            arci::gamepad::Axis::Unknown => Self::Unknown,
        }
    }
}

impl From<RAxis> for arci::gamepad::Axis {
    fn from(a: RAxis) -> Self {
        match a {
            RAxis::LeftStickX => Self::LeftStickX,
            RAxis::LeftStickY => Self::LeftStickY,
            RAxis::LeftTrigger => Self::LeftTrigger,
            RAxis::RightStickX => Self::RightStickX,
            RAxis::RightStickY => Self::RightStickY,
            RAxis::RightTrigger => Self::RightTrigger,
            RAxis::DPadX => Self::DPadX,
            RAxis::DPadY => Self::DPadY,
            RAxis::Unknown => Self::Unknown,
        }
    }
}

#[repr(C)]
#[derive(StableAbi)]
pub(crate) enum RGamepadEvent {
    ButtonPressed(RButton),
    ButtonReleased(RButton),
    AxisChanged(RAxis, RF64),
    Unknown,
}

impl From<arci::gamepad::GamepadEvent> for RGamepadEvent {
    fn from(e: arci::gamepad::GamepadEvent) -> Self {
        match e {
            arci::gamepad::GamepadEvent::ButtonPressed(b) => Self::ButtonPressed(b.into()),
            arci::gamepad::GamepadEvent::ButtonReleased(b) => Self::ButtonReleased(b.into()),
            arci::gamepad::GamepadEvent::AxisChanged(a, v) => Self::AxisChanged(a.into(), v.into()),
            arci::gamepad::GamepadEvent::Unknown => Self::Unknown,
        }
    }
}

impl From<RGamepadEvent> for arci::gamepad::GamepadEvent {
    fn from(e: RGamepadEvent) -> Self {
        match e {
            RGamepadEvent::ButtonPressed(b) => Self::ButtonPressed(b.into()),
            RGamepadEvent::ButtonReleased(b) => Self::ButtonReleased(b.into()),
            RGamepadEvent::AxisChanged(a, v) => Self::AxisChanged(a.into(), v.into()),
            RGamepadEvent::Unknown => Self::Unknown,
        }
    }
}

// =============================================================================
// Plugin

/// FFI-safe equivalent of [`Box<dyn Plugin>`](Plugin).
pub(crate) type PluginTraitObject = RPluginTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RPluginTrait: 'static {
    fn name(&self) -> RString;
    fn new_joint_trajectory_client(&self, args: RString) -> ROption<JointTrajectoryClientProxy>;
    fn new_speaker(&self, args: RString) -> ROption<SpeakerProxy>;
    fn new_move_base(&self, args: RString) -> ROption<MoveBaseProxy>;
    fn new_navigation(&self, args: RString) -> ROption<NavigationProxy>;
    fn new_localization(&self, args: RString) -> ROption<LocalizationProxy>;
    fn new_transform_resolver(&self, args: RString) -> ROption<TransformResolverProxy>;
    fn new_gamepad(&self, args: RString) -> ROption<GamepadProxy>;
}

impl<P> RPluginTrait for P
where
    P: ?Sized + Plugin,
{
    fn name(&self) -> RString {
        Plugin::name(self).into()
    }

    fn new_joint_trajectory_client(&self, args: RString) -> ROption<JointTrajectoryClientProxy> {
        Plugin::new_joint_trajectory_client(self, args.into())
            .map(JointTrajectoryClientProxy::new)
            .into()
    }

    fn new_speaker(&self, args: RString) -> ROption<SpeakerProxy> {
        Plugin::new_speaker(self, args.into())
            .map(SpeakerProxy::new)
            .into()
    }

    fn new_move_base(&self, args: RString) -> ROption<MoveBaseProxy> {
        Plugin::new_move_base(self, args.into())
            .map(MoveBaseProxy::new)
            .into()
    }

    fn new_navigation(&self, args: RString) -> ROption<NavigationProxy> {
        Plugin::new_navigation(self, args.into())
            .map(NavigationProxy::new)
            .into()
    }

    fn new_localization(&self, args: RString) -> ROption<LocalizationProxy> {
        Plugin::new_localization(self, args.into())
            .map(LocalizationProxy::new)
            .into()
    }

    fn new_transform_resolver(&self, args: RString) -> ROption<TransformResolverProxy> {
        Plugin::new_transform_resolver(self, args.into())
            .map(TransformResolverProxy::new)
            .into()
    }

    fn new_gamepad(&self, args: RString) -> ROption<GamepadProxy> {
        Plugin::new_gamepad(self, args.into())
            .map(GamepadProxy::new)
            .into()
    }
}

#[doc(hidden)]
#[repr(C)]
#[derive(StableAbi)]
#[sabi(kind(Prefix))]
#[sabi(missing_field(panic))]
pub struct PluginMod {
    #[sabi(last_prefix_field)]
    pub(crate) plugin_constructor: extern "C" fn() -> PluginProxy,
}

impl RootModule for PluginMod_Ref {
    const BASE_NAME: &'static str = "plugin";
    const NAME: &'static str = "plugin";
    const VERSION_STRINGS: VersionStrings = package_version_strings!();

    declare_root_module_statics!(PluginMod_Ref);
}

impl PluginMod_Ref {
    #[doc(hidden)]
    pub fn new(plugin_constructor: extern "C" fn() -> PluginProxy) -> Self {
        PluginMod { plugin_constructor }.leak_into_prefix()
    }
}
