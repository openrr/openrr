// This file is @generated by openrr-internal-codegen.
// It is not intended for manual editing.

#![allow(unused_variables)]
#![allow(clippy::useless_conversion, clippy::unit_arg)]

use abi_stable::{rtry, std_types::{RBox, RDuration, ROk, RResult, RStr}};
use super::*;
pub(crate) type PluginTraitObject = RPluginTrait_TO<RBox<()>>;
#[sabi_trait]
pub(crate) trait RPluginTrait: Send + Sync + 'static {
    fn new_gamepad(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::GamepadProxy>, RError>;
    fn new_joint_trajectory_client(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::JointTrajectoryClientProxy>, RError>;
    fn new_laser_scan2_d(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::LaserScan2DProxy>, RError>;
    fn new_localization(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::LocalizationProxy>, RError>;
    fn new_motor_drive_position(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::MotorDrivePositionProxy>, RError>;
    fn new_motor_drive_velocity(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::MotorDriveVelocityProxy>, RError>;
    fn new_motor_drive_effort(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::MotorDriveEffortProxy>, RError>;
    fn new_move_base(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::MoveBaseProxy>, RError>;
    fn new_navigation(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::NavigationProxy>, RError>;
    fn new_speaker(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::SpeakerProxy>, RError>;
    fn new_transform_resolver(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::TransformResolverProxy>, RError>;
}
impl<T> RPluginTrait for T
where
    T: crate::Plugin,
{
    fn new_gamepad(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::GamepadProxy>, RError> {
        ROk(
            rtry!(crate ::Plugin::new_gamepad(self, args.into()))
                .map(crate::GamepadProxy::new)
                .into(),
        )
    }
    fn new_joint_trajectory_client(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::JointTrajectoryClientProxy>, RError> {
        ROk(
            rtry!(crate ::Plugin::new_joint_trajectory_client(self, args.into()))
                .map(crate::JointTrajectoryClientProxy::new)
                .into(),
        )
    }
    fn new_laser_scan2_d(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::LaserScan2DProxy>, RError> {
        ROk(
            rtry!(crate ::Plugin::new_laser_scan2_d(self, args.into()))
                .map(crate::LaserScan2DProxy::new)
                .into(),
        )
    }
    fn new_localization(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::LocalizationProxy>, RError> {
        ROk(
            rtry!(crate ::Plugin::new_localization(self, args.into()))
                .map(crate::LocalizationProxy::new)
                .into(),
        )
    }
    fn new_motor_drive_position(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::MotorDrivePositionProxy>, RError> {
        ROk(
            rtry!(crate ::Plugin::new_motor_drive_position(self, args.into()))
                .map(crate::MotorDrivePositionProxy::new)
                .into(),
        )
    }
    fn new_motor_drive_velocity(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::MotorDriveVelocityProxy>, RError> {
        ROk(
            rtry!(crate ::Plugin::new_motor_drive_velocity(self, args.into()))
                .map(crate::MotorDriveVelocityProxy::new)
                .into(),
        )
    }
    fn new_motor_drive_effort(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::MotorDriveEffortProxy>, RError> {
        ROk(
            rtry!(crate ::Plugin::new_motor_drive_effort(self, args.into()))
                .map(crate::MotorDriveEffortProxy::new)
                .into(),
        )
    }
    fn new_move_base(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::MoveBaseProxy>, RError> {
        ROk(
            rtry!(crate ::Plugin::new_move_base(self, args.into()))
                .map(crate::MoveBaseProxy::new)
                .into(),
        )
    }
    fn new_navigation(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::NavigationProxy>, RError> {
        ROk(
            rtry!(crate ::Plugin::new_navigation(self, args.into()))
                .map(crate::NavigationProxy::new)
                .into(),
        )
    }
    fn new_speaker(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::SpeakerProxy>, RError> {
        ROk(
            rtry!(crate ::Plugin::new_speaker(self, args.into()))
                .map(crate::SpeakerProxy::new)
                .into(),
        )
    }
    fn new_transform_resolver(
        &self,
        args: RString,
    ) -> RResult<ROption<crate::TransformResolverProxy>, RError> {
        ROk(
            rtry!(crate ::Plugin::new_transform_resolver(self, args.into()))
                .map(crate::TransformResolverProxy::new)
                .into(),
        )
    }
}
pub(crate) type LaserScan2DTraitObject = RLaserScan2DTrait_TO<RBox<()>>;
#[abi_stable::sabi_trait]
pub(crate) trait RLaserScan2DTrait: Send + Sync + 'static {
    fn current_scan(&self) -> RResult<RScan2D, RError>;
}
impl<T> RLaserScan2DTrait for T
where
    T: arci::LaserScan2D + 'static,
{
    fn current_scan(&self) -> RResult<RScan2D, RError> {
        ROk(rtry!(arci::LaserScan2D::current_scan(self)).into())
    }
}
pub(crate) type LocalizationTraitObject = RLocalizationTrait_TO<RBox<()>>;
#[abi_stable::sabi_trait]
pub(crate) trait RLocalizationTrait: Send + Sync + 'static {
    fn current_pose(&self, frame_id: RStr<'_>) -> RResult<RIsometry2F64, RError>;
}
impl<T> RLocalizationTrait for T
where
    T: arci::Localization + 'static,
{
    fn current_pose(&self, frame_id: RStr<'_>) -> RResult<RIsometry2F64, RError> {
        ROk(rtry!(arci::Localization::current_pose(self, frame_id.into())).into())
    }
}
pub(crate) type MotorDrivePositionTraitObject = RMotorDrivePositionTrait_TO<RBox<()>>;
#[abi_stable::sabi_trait]
pub(crate) trait RMotorDrivePositionTrait: Send + Sync + 'static {
    fn set_motor_position(&self, position: f64) -> RResult<(), RError>;
    fn get_motor_position(&self) -> RResult<f64, RError>;
}
impl<T> RMotorDrivePositionTrait for T
where
    T: arci::MotorDrivePosition + 'static,
{
    fn set_motor_position(&self, position: f64) -> RResult<(), RError> {
        ROk(
            rtry!(arci::MotorDrivePosition::set_motor_position(self, position.into()))
                .into(),
        )
    }
    fn get_motor_position(&self) -> RResult<f64, RError> {
        ROk(rtry!(arci::MotorDrivePosition::get_motor_position(self)).into())
    }
}
pub(crate) type MotorDriveVelocityTraitObject = RMotorDriveVelocityTrait_TO<RBox<()>>;
#[abi_stable::sabi_trait]
pub(crate) trait RMotorDriveVelocityTrait: Send + Sync + 'static {
    fn set_motor_velocity(&self, velocity: f64) -> RResult<(), RError>;
    fn get_motor_velocity(&self) -> RResult<f64, RError>;
}
impl<T> RMotorDriveVelocityTrait for T
where
    T: arci::MotorDriveVelocity + 'static,
{
    fn set_motor_velocity(&self, velocity: f64) -> RResult<(), RError> {
        ROk(
            rtry!(arci::MotorDriveVelocity::set_motor_velocity(self, velocity.into()))
                .into(),
        )
    }
    fn get_motor_velocity(&self) -> RResult<f64, RError> {
        ROk(rtry!(arci::MotorDriveVelocity::get_motor_velocity(self)).into())
    }
}
pub(crate) type MotorDriveEffortTraitObject = RMotorDriveEffortTrait_TO<RBox<()>>;
#[abi_stable::sabi_trait]
pub(crate) trait RMotorDriveEffortTrait: Send + Sync + 'static {
    fn set_motor_effort(&self, effort: f64) -> RResult<(), RError>;
    fn get_motor_effort(&self) -> RResult<f64, RError>;
}
impl<T> RMotorDriveEffortTrait for T
where
    T: arci::MotorDriveEffort + 'static,
{
    fn set_motor_effort(&self, effort: f64) -> RResult<(), RError> {
        ROk(rtry!(arci::MotorDriveEffort::set_motor_effort(self, effort.into())).into())
    }
    fn get_motor_effort(&self) -> RResult<f64, RError> {
        ROk(rtry!(arci::MotorDriveEffort::get_motor_effort(self)).into())
    }
}
pub(crate) type MoveBaseTraitObject = RMoveBaseTrait_TO<RBox<()>>;
#[abi_stable::sabi_trait]
pub(crate) trait RMoveBaseTrait: Send + Sync + 'static {
    fn send_velocity(&self, velocity: RBaseVelocity) -> RResult<(), RError>;
    fn current_velocity(&self) -> RResult<RBaseVelocity, RError>;
}
impl<T> RMoveBaseTrait for T
where
    T: arci::MoveBase + 'static,
{
    fn send_velocity(&self, velocity: RBaseVelocity) -> RResult<(), RError> {
        ROk(rtry!(arci::MoveBase::send_velocity(self, & velocity.into())).into())
    }
    fn current_velocity(&self) -> RResult<RBaseVelocity, RError> {
        ROk(rtry!(arci::MoveBase::current_velocity(self)).into())
    }
}
pub(crate) type NavigationTraitObject = RNavigationTrait_TO<RBox<()>>;
#[abi_stable::sabi_trait]
pub(crate) trait RNavigationTrait: Send + Sync + 'static {
    fn send_goal_pose(
        &self,
        goal: RIsometry2F64,
        frame_id: RStr<'_>,
        timeout: RDuration,
    ) -> RResult<RBlockingWait, RError>;
    fn cancel(&self) -> RResult<(), RError>;
}
impl<T> RNavigationTrait for T
where
    T: arci::Navigation + 'static,
{
    fn send_goal_pose(
        &self,
        goal: RIsometry2F64,
        frame_id: RStr<'_>,
        timeout: RDuration,
    ) -> RResult<RBlockingWait, RError> {
        ROk(
            rtry!(
                arci::Navigation::send_goal_pose(self, goal.into(), frame_id.into(),
                timeout.into())
            )
                .into(),
        )
    }
    fn cancel(&self) -> RResult<(), RError> {
        ROk(rtry!(arci::Navigation::cancel(self)).into())
    }
}
pub(crate) type SpeakerTraitObject = RSpeakerTrait_TO<RBox<()>>;
#[abi_stable::sabi_trait]
pub(crate) trait RSpeakerTrait: Send + Sync + 'static {
    fn speak(&self, message: RStr<'_>) -> RResult<RBlockingWait, RError>;
}
impl<T> RSpeakerTrait for T
where
    T: arci::Speaker + 'static,
{
    fn speak(&self, message: RStr<'_>) -> RResult<RBlockingWait, RError> {
        ROk(rtry!(arci::Speaker::speak(self, message.into())).into())
    }
}
pub(crate) type TransformResolverTraitObject = RTransformResolverTrait_TO<RBox<()>>;
#[abi_stable::sabi_trait]
pub(crate) trait RTransformResolverTrait: Send + Sync + 'static {
    fn resolve_transformation(
        &self,
        from: RStr<'_>,
        to: RStr<'_>,
        time: RSystemTime,
    ) -> RResult<RIsometry3F64, RError>;
}
impl<T> RTransformResolverTrait for T
where
    T: arci::TransformResolver + 'static,
{
    fn resolve_transformation(
        &self,
        from: RStr<'_>,
        to: RStr<'_>,
        time: RSystemTime,
    ) -> RResult<RIsometry3F64, RError> {
        ROk(
            rtry!(
                arci::TransformResolver::resolve_transformation(self, from.into(), to
                .into(), rtry!(time.try_into()))
            )
                .into(),
        )
    }
}
/// FFI-safe equivalent of [`arci::TrajectoryPoint`].
#[derive(StableAbi)]
#[repr(C)]
pub(crate) struct RTrajectoryPoint {
    positions: RVec<f64>,
    velocities: ROption<RVec<f64>>,
    time_from_start: RDuration,
}
impl From<arci::TrajectoryPoint> for RTrajectoryPoint {
    fn from(v: arci::TrajectoryPoint) -> Self {
        let arci::TrajectoryPoint { positions, velocities, time_from_start } = v;
        Self {
            positions: positions.into_iter().collect(),
            velocities: velocities.map(|v| v.into_iter().collect()).into(),
            time_from_start: time_from_start.into(),
        }
    }
}
impl From<RTrajectoryPoint> for arci::TrajectoryPoint {
    fn from(v: RTrajectoryPoint) -> Self {
        let RTrajectoryPoint { positions, velocities, time_from_start } = v;
        Self {
            positions: positions.into_iter().collect(),
            velocities: velocities.into_option().map(|v| v.into_iter().collect()),
            time_from_start: time_from_start.into(),
        }
    }
}
/// FFI-safe equivalent of [`arci::Scan2D`].
#[derive(StableAbi)]
#[repr(C)]
pub(crate) struct RScan2D {
    angle_min: f64,
    angle_max: f64,
    angle_increment: f64,
    time_increment: f64,
    scan_time: f64,
    range_min: f64,
    range_max: f64,
    ranges: RVec<f64>,
    intensities: RVec<f64>,
}
impl From<arci::Scan2D> for RScan2D {
    fn from(v: arci::Scan2D) -> Self {
        let arci::Scan2D {
            angle_min,
            angle_max,
            angle_increment,
            time_increment,
            scan_time,
            range_min,
            range_max,
            ranges,
            intensities,
        } = v;
        Self {
            angle_min,
            angle_max,
            angle_increment,
            time_increment,
            scan_time,
            range_min,
            range_max,
            ranges: ranges.into_iter().collect(),
            intensities: intensities.into_iter().collect(),
        }
    }
}
impl From<RScan2D> for arci::Scan2D {
    fn from(v: RScan2D) -> Self {
        let RScan2D {
            angle_min,
            angle_max,
            angle_increment,
            time_increment,
            scan_time,
            range_min,
            range_max,
            ranges,
            intensities,
        } = v;
        Self {
            angle_min,
            angle_max,
            angle_increment,
            time_increment,
            scan_time,
            range_min,
            range_max,
            ranges: ranges.into_iter().collect(),
            intensities: intensities.into_iter().collect(),
        }
    }
}
/// FFI-safe equivalent of [`arci::BaseVelocity`].
#[derive(StableAbi)]
#[repr(C)]
pub(crate) struct RBaseVelocity {
    x: f64,
    y: f64,
    theta: f64,
}
impl From<arci::BaseVelocity> for RBaseVelocity {
    fn from(v: arci::BaseVelocity) -> Self {
        let arci::BaseVelocity { x, y, theta } = v;
        Self { x, y, theta }
    }
}
impl From<RBaseVelocity> for arci::BaseVelocity {
    fn from(v: RBaseVelocity) -> Self {
        let RBaseVelocity { x, y, theta } = v;
        Self { x, y, theta }
    }
}
/// FFI-safe equivalent of [`arci::Button`].
#[derive(StableAbi)]
#[repr(C)]
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
    fn from(v: arci::gamepad::Button) -> Self {
        match v {
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
    fn from(v: RButton) -> Self {
        match v {
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
/// FFI-safe equivalent of [`arci::Axis`].
#[derive(StableAbi)]
#[repr(C)]
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
    fn from(v: arci::gamepad::Axis) -> Self {
        match v {
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
    fn from(v: RAxis) -> Self {
        match v {
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
/// FFI-safe equivalent of [`arci::GamepadEvent`].
#[derive(StableAbi)]
#[repr(C)]
pub(crate) enum RGamepadEvent {
    ButtonPressed(RButton),
    ButtonReleased(RButton),
    AxisChanged(RAxis, f64),
    Connected,
    Disconnected,
    Unknown,
}
impl From<arci::gamepad::GamepadEvent> for RGamepadEvent {
    fn from(v: arci::gamepad::GamepadEvent) -> Self {
        match v {
            arci::gamepad::GamepadEvent::ButtonPressed(field0) => {
                Self::ButtonPressed(field0.into())
            }
            arci::gamepad::GamepadEvent::ButtonReleased(field0) => {
                Self::ButtonReleased(field0.into())
            }
            arci::gamepad::GamepadEvent::AxisChanged(field0, field1) => {
                Self::AxisChanged(field0.into(), field1)
            }
            arci::gamepad::GamepadEvent::Connected => Self::Connected,
            arci::gamepad::GamepadEvent::Disconnected => Self::Disconnected,
            arci::gamepad::GamepadEvent::Unknown => Self::Unknown,
        }
    }
}
impl From<RGamepadEvent> for arci::gamepad::GamepadEvent {
    fn from(v: RGamepadEvent) -> Self {
        match v {
            RGamepadEvent::ButtonPressed(field0) => Self::ButtonPressed(field0.into()),
            RGamepadEvent::ButtonReleased(field0) => Self::ButtonReleased(field0.into()),
            RGamepadEvent::AxisChanged(field0, field1) => {
                Self::AxisChanged(field0.into(), field1)
            }
            RGamepadEvent::Connected => Self::Connected,
            RGamepadEvent::Disconnected => Self::Disconnected,
            RGamepadEvent::Unknown => Self::Unknown,
        }
    }
}
