//! Experimental plugin support for [`arci`].

#![warn(missing_debug_implementations, missing_docs, rust_2018_idioms)]

mod proxy;

use std::{
    convert::TryInto,
    fmt,
    path::Path,
    sync::Arc,
    time::{Duration, SystemTime},
};

use abi_stable::{erased_types::TU_Opaque, library::lib_header_from_path, StableAbi};
use anyhow::Result;
use arci::{async_trait, Isometry2, Isometry3, WaitFuture};

// This is not a public API. Use export_plugin! macro for plugin exporting.
#[doc(hidden)]
pub use crate::proxy::PluginMod_Ref;
use crate::proxy::{
    GamepadTraitObject, JointTrajectoryClientTraitObject, LocalizationTraitObject,
    MoveBaseTraitObject, NavigationTraitObject, PluginTraitObject, SpeakerTraitObject,
    TransformResolverTraitObject,
};

/// Exports the plugin that will instantiated with the specified expression.
///
/// # Examples
///
/// ```
/// use openrr_plugin::Plugin;
///
/// openrr_plugin::export_plugin!(MyPlugin);
///
/// pub struct MyPlugin;
///
/// impl Plugin for MyPlugin {
///     fn name(&self) -> String {
///         "MyPlugin".into()
///     }
/// }
/// ```
#[macro_export]
macro_rules! export_plugin {
    ($plugin_constructor:expr $(,)?) => {
        /// Exports the root module of this library.
        ///
        /// This code isn't run until the layout of the type it returns is checked.
        #[::abi_stable::export_root_module]
        pub fn instantiate_root_module() -> $crate::PluginMod_Ref {
            $crate::PluginMod_Ref::new(plugin_constructor)
        }

        /// Instantiates the plugin.
        #[::abi_stable::sabi_extern_fn]
        pub fn plugin_constructor() -> $crate::PluginProxy {
            $crate::PluginProxy::new($plugin_constructor)
        }
    };
}

/// The plugin manager.
#[derive(Debug, Default)]
pub struct PluginManager {
    plugins: Vec<PluginProxy>,
}

impl PluginManager {
    /// Creates a new `PluginManager`.
    pub fn new() -> Self {
        Self::default()
    }

    /// Loads a plugin from the specified path.
    pub fn load(&mut self, path: impl AsRef<Path>) -> Result<()> {
        let path = path.as_ref();

        let header = lib_header_from_path(&path)?;
        let root_module = header.init_root_module::<PluginMod_Ref>()?;

        let plugin_constructor = root_module.plugin_constructor();
        let plugin = plugin_constructor();

        self.plugins.push(plugin);
        Ok(())
    }

    /// Returns the list of all plugins managed by this plugin manager.
    pub fn plugins(&self) -> &[PluginProxy] {
        &self.plugins
    }
}

/// The plugin trait.
pub trait Plugin: 'static {
    /// Returns the name of this plugin.
    ///
    /// NOTE: This is *not* a unique identifier.
    fn name(&self) -> String;

    /// Creates a new instance of [`arci::JointTrajectoryClient`] with the specified arguments.
    fn new_joint_trajectory_client(
        &self,
        args: String,
    ) -> Option<Box<dyn arci::JointTrajectoryClient>> {
        let _ = args;
        None
    }

    /// Creates a new instance of [`arci::Speaker`] with the specified arguments.
    fn new_speaker(&self, args: String) -> Option<Box<dyn arci::Speaker>> {
        let _ = args;
        None
    }

    /// Creates a new instance of [`arci::MoveBase`] with the specified arguments.
    fn new_move_base(&self, args: String) -> Option<Box<dyn arci::MoveBase>> {
        let _ = args;
        None
    }

    /// Creates a new instance of [`arci::Navigation`] with the specified arguments.
    fn new_navigation(&self, args: String) -> Option<Box<dyn arci::Navigation>> {
        let _ = args;
        None
    }

    /// Creates a new instance of [`arci::Localization`] with the specified arguments.
    fn new_localization(&self, args: String) -> Option<Box<dyn arci::Localization>> {
        let _ = args;
        None
    }

    /// Creates a new instance of [`arci::TransformResolver`] with the specified arguments.
    fn new_transform_resolver(&self, args: String) -> Option<Box<dyn arci::TransformResolver>> {
        let _ = args;
        None
    }

    /// Creates a new instance of [`arci::Gamepad`] with the specified arguments.
    fn new_gamepad(&self, args: String) -> Option<Box<dyn arci::Gamepad>> {
        let _ = args;
        None
    }
}

/// FFI-safe equivalent of [`Box<dyn Plugin>`](Plugin).
#[repr(C)]
#[derive(StableAbi)]
pub struct PluginProxy(PluginTraitObject);

impl PluginProxy {
    /// Creates a new `PluginProxy`.
    pub fn new<P>(plugin: P) -> Self
    where
        P: Plugin,
    {
        Self(PluginTraitObject::from_value(plugin, TU_Opaque))
    }

    /// Returns the name of this plugin.
    ///
    /// NOTE: This is *not* a unique identifier.
    pub fn name(&self) -> String {
        self.0.name().into()
    }

    /// Creates a new instance of [`arci::JointTrajectoryClient`] with the specified arguments.
    pub fn new_joint_trajectory_client(&self, args: String) -> Option<JointTrajectoryClientProxy> {
        self.0
            .new_joint_trajectory_client(args.into())
            .into_option()
    }

    /// Creates a new instance of [`arci::Speaker`] with the specified arguments.
    pub fn new_speaker(&self, args: String) -> Option<SpeakerProxy> {
        self.0.new_speaker(args.into()).into_option()
    }

    /// Creates a new instance of [`arci::MoveBase`] with the specified arguments.
    pub fn new_move_base(&self, args: String) -> Option<MoveBaseProxy> {
        self.0.new_move_base(args.into()).into_option()
    }

    /// Creates a new instance of [`arci::Navigation`] with the specified arguments.
    pub fn new_navigation(&self, args: String) -> Option<NavigationProxy> {
        self.0.new_navigation(args.into()).into_option()
    }

    /// Creates a new instance of [`arci::Localization`] with the specified arguments.
    pub fn new_localization(&self, args: String) -> Option<LocalizationProxy> {
        self.0.new_localization(args.into()).into_option()
    }

    /// Creates a new instance of [`arci::TransformResolver`] with the specified arguments.
    pub fn new_transform_resolver(&self, args: String) -> Option<TransformResolverProxy> {
        self.0.new_transform_resolver(args.into()).into_option()
    }

    /// Creates a new instance of [`arci::Gamepad`] with the specified arguments.
    pub fn new_gamepad(&self, args: String) -> Option<GamepadProxy> {
        self.0.new_gamepad(args.into()).into_option()
    }
}

impl fmt::Debug for PluginProxy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PluginProxy")
            .field("name", &self.name())
            .finish()
    }
}

// =============================================================================
// JointTrajectoryClient

/// FFI-safe equivalent of [`Box<dyn arci::JointTrajectoryClient>`](arci::JointTrajectoryClient).
#[repr(C)]
#[derive(StableAbi)]
pub struct JointTrajectoryClientProxy(JointTrajectoryClientTraitObject);

impl JointTrajectoryClientProxy {
    /// Creates a new `JointTrajectoryClientProxy`.
    pub fn new<T>(client: T) -> Self
    where
        T: arci::JointTrajectoryClient + 'static,
    {
        Self(JointTrajectoryClientTraitObject::from_value(
            client, TU_Opaque,
        ))
    }
}

impl arci::JointTrajectoryClient for JointTrajectoryClientProxy {
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
    ) -> Result<WaitFuture, arci::Error> {
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
    ) -> Result<WaitFuture, arci::Error> {
        Ok(self
            .0
            .send_joint_trajectory(trajectory.into_iter().map(Into::into).collect())
            .into_result()?
            .into())
    }
}

impl fmt::Debug for JointTrajectoryClientProxy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("JointTrajectoryClientProxy").finish()
    }
}

// =============================================================================
// arci::Speaker

/// FFI-safe equivalent of [`Box<dyn arci::Speaker>`](arci::Speaker).
#[repr(C)]
#[derive(StableAbi)]
pub struct SpeakerProxy(SpeakerTraitObject);

impl SpeakerProxy {
    /// Creates a new `SpeakerProxy`.
    pub fn new<T>(speaker: T) -> Self
    where
        T: arci::Speaker + 'static,
    {
        Self(SpeakerTraitObject::from_value(speaker, TU_Opaque))
    }
}

impl arci::Speaker for SpeakerProxy {
    fn speak(&self, message: &str) -> Result<WaitFuture, arci::Error> {
        Ok(self.0.speak(message.into()).into_result()?.into())
    }
}

impl fmt::Debug for SpeakerProxy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SpeakerProxy").finish()
    }
}

// =============================================================================
// arci::MoveBase

/// FFI-safe equivalent of [`Box<dyn arci::MoveBase>`](arci::MoveBase).
#[repr(C)]
#[derive(StableAbi)]
pub struct MoveBaseProxy(MoveBaseTraitObject);

impl MoveBaseProxy {
    /// Creates a new `MoveBaseProxy`.
    pub fn new<T>(base: T) -> Self
    where
        T: arci::MoveBase + 'static,
    {
        Self(MoveBaseTraitObject::from_value(base, TU_Opaque))
    }
}

impl arci::MoveBase for MoveBaseProxy {
    fn send_velocity(&self, velocity: &arci::BaseVelocity) -> Result<(), arci::Error> {
        self.0.send_velocity((*velocity).into()).into_result()?;
        Ok(())
    }

    fn current_velocity(&self) -> Result<arci::BaseVelocity, arci::Error> {
        Ok(self.0.current_velocity().into_result()?.into())
    }
}

impl fmt::Debug for MoveBaseProxy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MoveBaseProxy").finish()
    }
}

// =============================================================================
// arci::Navigation

/// FFI-safe equivalent of [`Box<dyn arci::Navigation>`](arci::Navigation).
#[repr(C)]
#[derive(StableAbi)]
pub struct NavigationProxy(NavigationTraitObject);

impl NavigationProxy {
    /// Creates a new `NavigationProxy`.
    pub fn new<T>(nav: T) -> Self
    where
        T: arci::Navigation + 'static,
    {
        Self(NavigationTraitObject::from_value(nav, TU_Opaque))
    }
}

impl arci::Navigation for NavigationProxy {
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: Duration,
    ) -> Result<WaitFuture, arci::Error> {
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

impl fmt::Debug for NavigationProxy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("NavigationProxy").finish()
    }
}

// =============================================================================
// arci::Localization

/// FFI-safe equivalent of [`Box<dyn arci::Localization>`](arci::Localization).
#[repr(C)]
#[derive(StableAbi)]
pub struct LocalizationProxy(LocalizationTraitObject);

impl LocalizationProxy {
    /// Creates a new `LocalizationProxy`.
    pub fn new<T>(loc: T) -> Self
    where
        T: arci::Localization + 'static,
    {
        Self(LocalizationTraitObject::from_value(loc, TU_Opaque))
    }
}

impl arci::Localization for LocalizationProxy {
    fn current_pose(&self, frame_id: &str) -> Result<Isometry2<f64>, arci::Error> {
        Ok(self.0.current_pose(frame_id.into()).into_result()?.into())
    }
}

impl fmt::Debug for LocalizationProxy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LocalizationProxy").finish()
    }
}

// =============================================================================
// arci::TransformResolver

/// FFI-safe equivalent of [`Box<dyn arci::TransformResolver>`](arci::TransformResolver).
#[repr(C)]
#[derive(StableAbi)]
pub struct TransformResolverProxy(TransformResolverTraitObject);

impl TransformResolverProxy {
    /// Creates a new `TransformResolverProxy`.
    pub fn new<T>(resolver: T) -> Self
    where
        T: arci::TransformResolver + 'static,
    {
        Self(TransformResolverTraitObject::from_value(
            resolver, TU_Opaque,
        ))
    }
}

impl arci::TransformResolver for TransformResolverProxy {
    fn resolve_transformation(
        &self,
        from: &str,
        to: &str,
        time: SystemTime,
    ) -> Result<Isometry3<f64>, arci::Error> {
        Ok(self
            .0
            .resolve_transformation(from.into(), to.into(), time.try_into()?)
            .into_result()?
            .into())
    }
}

impl fmt::Debug for TransformResolverProxy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TransformResolverProxy").finish()
    }
}

// =============================================================================
// arci::Gamepad

/// FFI-safe equivalent of [`Box<dyn arci::Gamepad>`](arci::Gamepad).
// Don't implement Clone -- use of Arc is implementation detail.
#[repr(C)]
#[derive(StableAbi)]
pub struct GamepadProxy(GamepadTraitObject);

impl GamepadProxy {
    /// Creates a new `GamepadProxy`.
    pub fn new<T>(gamepad: T) -> Self
    where
        T: arci::Gamepad + 'static,
    {
        Self(GamepadTraitObject::from_value(Arc::new(gamepad), TU_Opaque))
    }
}

#[async_trait]
impl arci::Gamepad for GamepadProxy {
    async fn next_event(&self) -> arci::gamepad::GamepadEvent {
        let this = Self(self.0.clone());
        tokio::task::spawn_blocking(move || this.0.next_event().into())
            .await
            .unwrap_or(arci::gamepad::GamepadEvent::Unknown)
    }

    fn stop(&self) {
        self.0.stop();
    }
}

impl fmt::Debug for GamepadProxy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GamepadProxy").finish()
    }
}
