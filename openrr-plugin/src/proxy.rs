//! This module defines FFI-safe equivalents of the various types and traits used in arci.

use std::{future::Future, sync::Arc};

use abi_stable::{
    erased_types::TU_Opaque,
    sabi_trait,
    std_types::{RBox, RDuration, RErr, ROk, ROption, RStr, RString, RVec},
    StableAbi,
};
use num_traits::Float;

use crate::{RError, RResult, StaticJointTrajectoryClient};

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
#[repr(C)]
#[derive(Debug, Clone, Copy, StableAbi)]
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
        // https://docs.rs/num-traits/0.2/num_traits/float/trait.Float.html#tymethod.integer_decode
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
// nalgebra::Isometry2<f64>

/// FFI-safe equivalent of [`nalgebra::Isometry2<f64>`](nalgebra::Isometry2).
#[repr(C)]
#[derive(StableAbi, Clone, Copy, Debug)]
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
#[derive(StableAbi, Clone, Copy, Debug)]
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
#[derive(StableAbi, Clone, Copy, Debug)]
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

/* used in TransformResolver
// =============================================================================
// nalgebra::Isometry3<f64>

/// FFI-safe equivalent of [`nalgebra::Isometry3<f64>`](nalgebra::Isometry3).
#[repr(C)]
#[derive(StableAbi, Clone, Copy, Debug)]
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
#[derive(StableAbi, Clone, Copy, Debug)]
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
#[derive(StableAbi, Clone, Copy, Debug)]
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

*/

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

#[sabi_trait]
trait RWaitTrait: Send + 'static {
    fn wait(self) -> RResult<()>;
}

type RBoxWait = RWaitTrait_TO<RBox<()>>;

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

/// FFI-safe equivalent of [`Arc<dyn arci::JointTrajectoryClient>`](arci::JointTrajectoryClient).
pub(crate) type AbiStableJointTrajectoryClientTraitObj = RJointTrajectoryClientTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RJointTrajectoryClientTrait: Send + Sync + Clone + 'static {
    fn joint_names(&self) -> RVec<RString>;
    fn current_joint_positions(&self) -> RResult<RVec<RF64>>;
    fn send_joint_positions(
        &self,
        positions: RVec<RF64>,
        duration: RDuration,
    ) -> RResult<RBlockingWait>;
    fn send_joint_trajectory(&self, trajectory: RVec<RTrajectoryPoint>) -> RResult<RBlockingWait>;
}

impl<C> RJointTrajectoryClientTrait for Arc<C>
where
    C: ?Sized + StaticJointTrajectoryClient,
{
    fn joint_names(&self) -> RVec<RString> {
        StaticJointTrajectoryClient::joint_names(&**self)
            .into_iter()
            .map(|s| s.into())
            .collect()
    }

    fn current_joint_positions(&self) -> RResult<RVec<RF64>> {
        match StaticJointTrajectoryClient::current_joint_positions(&**self) {
            Ok(p) => ROk(p.into_iter().map(RF64::from).collect()),
            Err(e) => RErr(e.into()),
        }
    }

    fn send_joint_positions(
        &self,
        positions: RVec<RF64>,
        duration: RDuration,
    ) -> RResult<RBlockingWait> {
        match StaticJointTrajectoryClient::send_joint_positions(
            &**self,
            positions.into_iter().map(f64::from).collect(),
            duration.into(),
        ) {
            Ok(wait) => ROk(wait.into()),
            Err(e) => RErr(e.into()),
        }
    }

    fn send_joint_trajectory(&self, trajectory: RVec<RTrajectoryPoint>) -> RResult<RBlockingWait> {
        match StaticJointTrajectoryClient::send_joint_trajectory(
            &**self,
            trajectory
                .into_iter()
                .map(arci::TrajectoryPoint::from)
                .collect(),
        ) {
            Ok(wait) => ROk(wait.into()),
            Err(e) => RErr(e.into()),
        }
    }
}

/// FFI-safe equivalent of [`arci::TrajectoryPoint`].
#[repr(C)]
#[derive(StableAbi, Clone, Debug)]
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

/// FFI-safe equivalent of [`Arc<dyn arci::Speaker>`](arci::Speaker).
pub(crate) type AbiStableSpeakerTraitObj = RSpeakerTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RSpeakerTrait: Send + Sync + Clone + 'static {
    fn speak(&self, message: RStr<'_>) -> RResult<RBlockingWait>;
}

impl<T> RSpeakerTrait for Arc<T>
where
    T: ?Sized + arci::Speaker + 'static,
{
    fn speak(&self, message: RStr<'_>) -> RResult<RBlockingWait> {
        match arci::Speaker::speak(&**self, message.into()) {
            Ok(wait) => ROk(wait.into()),
            Err(e) => RErr(e.into()),
        }
    }
}

// =============================================================================
// arci::MoveBase

/// FFI-safe equivalent of [`Arc<dyn arci::MoveBase>`](arci::MoveBase).
pub(crate) type AbiStableMoveBaseTraitObj = RMoveBaseTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RMoveBaseTrait: Send + Sync + Clone + 'static {
    fn send_velocity(&self, velocity: &RBaseVelocity) -> RResult<()>;
    fn current_velocity(&self) -> RResult<RBaseVelocity>;
}

impl<T> RMoveBaseTrait for Arc<T>
where
    T: ?Sized + arci::MoveBase + 'static,
{
    fn send_velocity(&self, velocity: &RBaseVelocity) -> RResult<()> {
        match arci::MoveBase::send_velocity(&**self, &arci::BaseVelocity::from(*velocity)) {
            Ok(()) => ROk(()),
            Err(e) => RErr(e.into()),
        }
    }

    fn current_velocity(&self) -> RResult<RBaseVelocity> {
        match arci::MoveBase::current_velocity(&**self) {
            Ok(v) => ROk(v.into()),
            Err(e) => RErr(e.into()),
        }
    }
}

/// FFI-safe equivalent of [`arci::BaseVelocity`].
#[repr(C)]
#[derive(StableAbi, Clone, Copy)]
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

/// FFI-safe equivalent of [`Arc<dyn arci::Navigation>`](arci::Navigation).
pub(crate) type AbiStableNavigationTraitObj = RNavigationTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RNavigationTrait: Send + Sync + Clone + 'static {
    fn send_goal_pose(
        &self,
        goal: RIsometry2F64,
        frame_id: RStr<'_>,
        timeout: RDuration,
    ) -> RResult<RBlockingWait>;

    fn cancel(&self) -> RResult<()>;
}

impl<T> RNavigationTrait for Arc<T>
where
    T: ?Sized + arci::Navigation + 'static,
{
    fn send_goal_pose(
        &self,
        goal: RIsometry2F64,
        frame_id: RStr<'_>,
        timeout: RDuration,
    ) -> RResult<RBlockingWait> {
        match arci::Navigation::send_goal_pose(
            &**self,
            goal.into(),
            frame_id.into(),
            timeout.into(),
        ) {
            Ok(wait) => ROk(wait.into()),
            Err(e) => RErr(e.into()),
        }
    }

    fn cancel(&self) -> RResult<()> {
        match arci::Navigation::cancel(&**self) {
            Ok(()) => ROk(()),
            Err(e) => RErr(e.into()),
        }
    }
}

// =============================================================================
// arci::Localization

/// FFI-safe equivalent of [`Arc<dyn arci::Localization>`](arci::Localization).
pub(crate) type AbiStableLocalizationTraitObj = RLocalizationTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RLocalizationTrait: Send + Sync + Clone + 'static {
    fn current_pose(&self, frame_id: RStr<'_>) -> RResult<RIsometry2F64>;
}

impl<T> RLocalizationTrait for Arc<T>
where
    T: ?Sized + arci::Localization + 'static,
{
    fn current_pose(&self, frame_id: RStr<'_>) -> RResult<RIsometry2F64> {
        match arci::Localization::current_pose(&**self, frame_id.into()) {
            Ok(pose) => ROk(pose.into()),
            Err(e) => RErr(e.into()),
        }
    }
}

/* TODO: We need a FFI-safe equivalent of SystemTime.
// =============================================================================
// arci::TransformResolver

/// FFI-safe equivalent of [`Arc<dyn arci::TransformResolver>`](arci::TransformResolver).
pub(crate) type AbiStableTransformResolverTraitObj = RTransformResolverTrait_TO<RBox<()>>;

#[sabi_trait]
pub(crate) trait RTransformResolverTrait: Send + Sync + Clone + 'static {
    fn resolve_transformation(
        &self,
        from: RStr<'_>,
        to: RStr<'_>,
        time: RSystemTime,
    ) -> RResult<RIsometry3F64>;
}

impl<T> RTransformResolverTrait for Arc<T>
where
    T: ?Sized + arci::TransformResolver + 'static,
{
    fn resolve_transformation(
        &self,
        from: RStr<'_>,
        to: RStr<'_>,
        time: RSystemTime,
    ) -> RResult<RIsometry3F64> {
        match arci::TransformResolver::resolve_transformation(
            &**self,
            from.into(),
            to.into(),
            time.into(),
        ) {
            Ok(pose) => ROk(pose.into()),
            Err(e) => RErr(e.into()),
        }
    }
}
*/

// =============================================================================
// arci::Gamepad

/// FFI-safe equivalent of [`Arc<dyn arci::Gamepad>`](arci::Gamepad).
pub(crate) type AbiStableGamepadTraitObj = RGamepadTrait_TO<RBox<()>>;

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
#[derive(StableAbi, Clone, Copy)]
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
#[derive(StableAbi, Clone, Copy)]
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
#[derive(StableAbi, Clone)]
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
