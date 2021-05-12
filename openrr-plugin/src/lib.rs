//! Experimental plugin support for [`arci`].

#![warn(rust_2018_idioms)]

mod proxy;

use std::{fmt, path::Path, sync::Arc, time::Duration};

use abi_stable::{
    declare_root_module_statics,
    erased_types::TU_Opaque,
    library::{lib_header_from_path, RootModule},
    package_version_strings, sabi_trait,
    sabi_types::VersionStrings,
    std_types::{RBox, RBoxError, ROption, RString, RVec},
    StableAbi,
};
use anyhow::{format_err, Result};
pub use arci;
use arci::{async_trait, Isometry2, WaitFuture};

use crate::proxy::{
    AbiStableGamepadTraitObj, AbiStableJointTrajectoryClientTraitObj,
    AbiStableLocalizationTraitObj, AbiStableMoveBaseTraitObj, AbiStableNavigationTraitObj,
    AbiStableSpeakerTraitObj,
};

pub type RResult<T, E = RError> = abi_stable::std_types::RResult<T, E>;

#[macro_export]
macro_rules! export_plugin {
    ($plugin_constructor:expr $(,)?) => {
        /// Exports the root module of this library.
        ///
        /// This code isn't run until the layout of the type it returns is checked.
        #[::abi_stable::export_root_module]
        pub fn instantiate_root_module() -> $crate::PluginMod_Ref {
            ::abi_stable::prefix_type::PrefixTypeTrait::leak_into_prefix($crate::PluginMod { new })
        }

        /// Instantiates the plugin.
        #[::abi_stable::sabi_extern_fn]
        pub fn new(
            __args: ::abi_stable::std_types::RVec<::abi_stable::std_types::RString>,
        ) -> $crate::RResult<$crate::RPlugin> {
            match ($plugin_constructor)(
                __args
                    .into_iter()
                    .map(::std::convert::Into::into)
                    .collect::<::std::vec::Vec<::std::string::String>>(),
            ) {
                ::std::result::Result::Ok(plugin) => {
                    ::abi_stable::std_types::RResult::ROk($crate::RPlugin::new(plugin))
                }
                ::std::result::Result::Err(e) => {
                    ::abi_stable::std_types::RResult::RErr($crate::RError::from(e))
                }
            }
        }
    };
}

#[derive(Default)]
pub struct PluginManager {
    plugins: Vec<RPlugin>,
}

impl PluginManager {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn load(&mut self, path: impl AsRef<Path>) -> Result<()> {
        let path = path.as_ref();

        let header = lib_header_from_path(&path)?;
        let root_module = header.init_root_module::<PluginMod_Ref>()?;

        let plugin_constructor = root_module.new();
        let plugin = plugin_constructor(RVec::new()).into_result()?;

        self.plugins.push(plugin);
        Ok(())
    }

    pub fn plugins(&self) -> &[RPlugin] {
        &self.plugins
    }
}

#[doc(hidden)]
#[repr(C)]
#[derive(StableAbi)]
#[sabi(kind(Prefix))]
#[sabi(missing_field(panic))]
pub struct PluginMod {
    #[sabi(last_prefix_field)]
    pub new: extern "C" fn(RVec<RString>) -> RResult<RPlugin>,
}

impl RootModule for PluginMod_Ref {
    const BASE_NAME: &'static str = "plugin";
    const NAME: &'static str = "plugin";
    const VERSION_STRINGS: VersionStrings = package_version_strings!();

    declare_root_module_statics!(PluginMod_Ref);
}

/// FFI-safe equivalent of [`Box<dyn Plugin>`](Plugin).
#[repr(C)]
#[derive(StableAbi)]
pub struct RPlugin(RBoxPluginTrait);

impl RPlugin {
    pub fn new<P>(plugin: P) -> Self
    where
        P: Plugin,
    {
        Self(RBoxPluginTrait::from_value(plugin, TU_Opaque))
    }

    pub fn name(&self) -> String {
        self.0.name().into()
    }

    pub fn joint_trajectory_client(&self) -> Option<RJointTrajectoryClient> {
        self.0.joint_trajectory_client().into_option()
    }

    pub fn speaker(&self) -> Option<RSpeaker> {
        self.0.speaker().into_option()
    }

    pub fn move_base(&self) -> Option<RMoveBase> {
        self.0.move_base().into_option()
    }

    pub fn navigation(&self) -> Option<RNavigation> {
        self.0.navigation().into_option()
    }

    pub fn localization(&self) -> Option<RLocalization> {
        self.0.localization().into_option()
    }

    pub fn gamepad(&self) -> Option<RGamepad> {
        self.0.gamepad().into_option()
    }
}

pub trait Plugin: 'static {
    fn name(&self) -> String;
    // TODO: multiple JointTrajectoryClient?
    fn joint_trajectory_client(&self) -> Option<Arc<dyn StaticJointTrajectoryClient>> {
        None
    }
    fn speaker(&self) -> Option<Arc<dyn arci::Speaker>> {
        None
    }
    fn move_base(&self) -> Option<Arc<dyn arci::MoveBase>> {
        None
    }
    fn navigation(&self) -> Option<Arc<dyn arci::Navigation>> {
        None
    }
    fn localization(&self) -> Option<Arc<dyn arci::Localization>> {
        None
    }
    fn gamepad(&self) -> Option<Arc<dyn arci::Gamepad>> {
        None
    }
}

/// FFI-safe equivalent of [`Plugin`] trait.
#[sabi_trait]
trait RPluginTrait: 'static {
    fn name(&self) -> RString;
    fn joint_trajectory_client(&self) -> ROption<RJointTrajectoryClient>;
    fn speaker(&self) -> ROption<RSpeaker>;
    fn move_base(&self) -> ROption<RMoveBase>;
    fn navigation(&self) -> ROption<RNavigation>;
    fn localization(&self) -> ROption<RLocalization>;
    fn gamepad(&self) -> ROption<RGamepad>;
}

type RBoxPluginTrait = RPluginTrait_TO<RBox<()>>;

impl<P> RPluginTrait for P
where
    P: ?Sized + Plugin,
{
    fn name(&self) -> RString {
        Plugin::name(self).into()
    }

    fn joint_trajectory_client(&self) -> ROption<RJointTrajectoryClient> {
        Plugin::joint_trajectory_client(self)
            .map(RJointTrajectoryClient::new)
            .into()
    }

    fn speaker(&self) -> ROption<RSpeaker> {
        Plugin::speaker(self).map(RSpeaker::new).into()
    }

    fn move_base(&self) -> ROption<RMoveBase> {
        Plugin::move_base(self).map(RMoveBase::new).into()
    }

    fn navigation(&self) -> ROption<RNavigation> {
        Plugin::navigation(self).map(RNavigation::new).into()
    }

    fn localization(&self) -> ROption<RLocalization> {
        Plugin::localization(self).map(RLocalization::new).into()
    }

    fn gamepad(&self) -> ROption<RGamepad> {
        Plugin::gamepad(self).map(RGamepad::new).into()
    }
}

// =============================================================================
// Error

/// FFI-safe equivalent of [`arci::Error`].
#[repr(C)]
#[derive(StableAbi)]
pub struct RError {
    repr: RBoxError,
}

impl From<arci::Error> for RError {
    fn from(e: arci::Error) -> Self {
        Self {
            // TODO: propagate error kind.
            repr: RBoxError::from_box(Box::new(e)),
        }
    }
}

impl From<RError> for arci::Error {
    fn from(e: RError) -> Self {
        // TODO: propagate error kind.
        Self::Other(format_err!("{}", e.repr))
    }
}

impl fmt::Debug for RError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(&self.repr, f)
    }
}

impl fmt::Display for RError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(&self.repr, f)
    }
}

impl std::error::Error for RError {}

// =============================================================================
// JointTrajectoryClient

/// FFI-safe equivalent of [`Arc<dyn arci::JointTrajectoryClient>`](arci::JointTrajectoryClient).
#[repr(C)]
#[derive(StableAbi, Clone)]
pub struct RJointTrajectoryClient(AbiStableJointTrajectoryClientTraitObj);

impl RJointTrajectoryClient {
    pub fn new<T>(client: Arc<T>) -> Self
    where
        T: ?Sized + StaticJointTrajectoryClient,
    {
        Self(AbiStableJointTrajectoryClientTraitObj::from_value(
            client, TU_Opaque,
        ))
    }
}

impl StaticJointTrajectoryClient for RJointTrajectoryClient {
    fn joint_names(&self) -> Vec<String> {
        self.0.joint_names().into_iter().map(|s| s.into()).collect()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        Ok(self
            .0
            .current_joint_positions()
            .into_result()?
            .into_iter()
            .map(f64::from)
            .collect())
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture<'static>, arci::Error> {
        Ok(self
            .0
            .send_joint_positions(
                positions.into_iter().map(Into::into).collect(),
                duration.into(),
            )
            .into_result()?
            .into())
    }

    fn send_joint_trajectory(
        &self,
        trajectory: Vec<arci::TrajectoryPoint>,
    ) -> Result<WaitFuture<'static>, arci::Error> {
        Ok(self
            .0
            .send_joint_trajectory(trajectory.into_iter().map(Into::into).collect())
            .into_result()?
            .into())
    }
}

impl arci::JointTrajectoryClient for RJointTrajectoryClient {
    fn joint_names(&self) -> Vec<String> {
        StaticJointTrajectoryClient::joint_names(self)
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        StaticJointTrajectoryClient::current_joint_positions(self)
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture<'static>, arci::Error> {
        StaticJointTrajectoryClient::send_joint_positions(self, positions, duration)
    }

    fn send_joint_trajectory(
        &self,
        trajectory: Vec<arci::TrajectoryPoint>,
    ) -> Result<WaitFuture<'static>, arci::Error> {
        StaticJointTrajectoryClient::send_joint_trajectory(self, trajectory)
    }
}

/// Almost equivalent to [`arci::JointTrajectoryClient`], but is `'static` and
/// returns `WaitFuture<'static>`.
pub trait StaticJointTrajectoryClient: Send + Sync + 'static {
    fn joint_names(&self) -> Vec<String>;
    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error>;
    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture<'static>, arci::Error>;
    fn send_joint_trajectory(
        &self,
        trajectory: Vec<arci::TrajectoryPoint>,
    ) -> Result<WaitFuture<'static>, arci::Error>;
}

// =============================================================================
// arci::Speaker

/// FFI-safe equivalent of [`Arc<dyn arci::Speaker>`](arci::Speaker).
#[repr(C)]
#[derive(StableAbi, Clone)]
pub struct RSpeaker(AbiStableSpeakerTraitObj);

impl RSpeaker {
    pub fn new<T>(speaker: Arc<T>) -> Self
    where
        T: ?Sized + arci::Speaker + 'static,
    {
        Self(AbiStableSpeakerTraitObj::from_value(speaker, TU_Opaque))
    }
}

impl arci::Speaker for RSpeaker {
    fn speak(&self, message: &str) -> Result<WaitFuture<'static>, arci::Error> {
        Ok(self.0.speak(message.into()).into_result()?.into())
    }
}

// =============================================================================
// arci::MoveBase

/// FFI-safe equivalent of [`Arc<dyn arci::MoveBase>`](arci::MoveBase).
#[repr(C)]
#[derive(StableAbi, Clone)]
pub struct RMoveBase(AbiStableMoveBaseTraitObj);

impl RMoveBase {
    pub fn new<T>(base: Arc<T>) -> Self
    where
        T: ?Sized + arci::MoveBase + 'static,
    {
        Self(AbiStableMoveBaseTraitObj::from_value(base, TU_Opaque))
    }
}

impl arci::MoveBase for RMoveBase {
    fn send_velocity(&self, velocity: &arci::BaseVelocity) -> Result<(), arci::Error> {
        self.0.send_velocity(&(*velocity).into()).into_result()?;
        Ok(())
    }

    fn current_velocity(&self) -> Result<arci::BaseVelocity, arci::Error> {
        Ok(self.0.current_velocity().into_result()?.into())
    }
}

// =============================================================================
// arci::Navigation

/// FFI-safe equivalent of [`Arc<dyn arci::Navigation>`](arci::Navigation).
#[repr(C)]
#[derive(StableAbi, Clone)]
pub struct RNavigation(AbiStableNavigationTraitObj);

impl RNavigation {
    pub fn new<T>(nav: Arc<T>) -> Self
    where
        T: ?Sized + arci::Navigation + 'static,
    {
        Self(AbiStableNavigationTraitObj::from_value(nav, TU_Opaque))
    }
}

impl arci::Navigation for RNavigation {
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: Duration,
    ) -> Result<WaitFuture<'static>, arci::Error> {
        Ok(self
            .0
            .send_goal_pose(goal.into(), frame_id.into(), timeout.into())
            .into_result()?
            .into())
    }

    fn cancel(&self) -> Result<(), arci::Error> {
        self.0.cancel().into_result()?;
        Ok(())
    }
}

// =============================================================================
// arci::Localization

/// FFI-safe equivalent of [`Arc<dyn arci::Localization>`](arci::Localization).
#[repr(C)]
#[derive(StableAbi, Clone)]
pub struct RLocalization(AbiStableLocalizationTraitObj);

impl RLocalization {
    pub fn new<T>(loc: Arc<T>) -> Self
    where
        T: ?Sized + arci::Localization + 'static,
    {
        Self(AbiStableLocalizationTraitObj::from_value(loc, TU_Opaque))
    }
}

impl arci::Localization for RLocalization {
    fn current_pose(&self, frame_id: &str) -> Result<Isometry2<f64>, arci::Error> {
        Ok(self.0.current_pose(frame_id.into()).into_result()?.into())
    }
}

/* TODO: We need a FFI-safe equivalent of SystemTime.
// =============================================================================
// arci::TransformResolver

/// FFI-safe equivalent of [`Arc<dyn arci::TransformResolver>`](arci::TransformResolver).
#[repr(C)]
#[derive(StableAbi, Clone)]
pub struct RTransformResolver(AbiStableTransformResolverTraitObj);

impl RTransformResolver {
    pub fn new<T>(resolver: Arc<T>) -> Self
    where
        T: ?Sized + arci::TransformResolver + 'static,
    {
        Self(AbiStableTransformResolverTraitObj::from_value(resolver, TU_Opaque))
    }
}

impl arci::TransformResolver for RTransformResolver {
    fn resolve_transformation(
        &self,
        from: &str,
        to: &str,
        time: SystemTime,
    ) -> Result<Isometry3<f64>, arci::Error> {
        Ok(self
            .0
            .resolve_transformation(from.into(), to.into(), time.into())
            .into_result()?
            .into())
    }
}
*/

// =============================================================================
// arci::Gamepad

/// FFI-safe equivalent of [`Arc<dyn arci::Gamepad>`](arci::Gamepad).
#[repr(C)]
#[derive(StableAbi, Clone)]
pub struct RGamepad(AbiStableGamepadTraitObj);

impl RGamepad {
    pub fn new<T>(loc: Arc<T>) -> Self
    where
        T: ?Sized + arci::Gamepad + 'static,
    {
        Self(AbiStableGamepadTraitObj::from_value(loc, TU_Opaque))
    }
}

#[async_trait]
impl arci::Gamepad for RGamepad {
    async fn next_event(&self) -> arci::gamepad::GamepadEvent {
        let this = self.clone();
        tokio::task::spawn_blocking(move || this.0.next_event().into())
            .await
            .unwrap_or(arci::gamepad::GamepadEvent::Unknown)
    }

    fn stop(&self) {
        self.0.stop();
    }
}
