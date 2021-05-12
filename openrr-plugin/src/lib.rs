//! Experimental plugin support for [`arci`].

#![warn(rust_2018_idioms)]

use std::{fmt, path::Path, sync::Arc, time::Duration};

use abi_stable::{
    declare_root_module_statics,
    erased_types::TU_Opaque,
    library::{lib_header_from_path, RootModule},
    package_version_strings, sabi_trait,
    sabi_types::VersionStrings,
    std_types::{RBox, RBoxError, RDuration, RErr, ROk, ROption, RResult, RStr, RString, RVec},
    StableAbi,
};
use anyhow::{format_err, Result};
pub use arci;
use arci::WaitFuture;
use num_traits::Float;

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
        ) -> ::abi_stable::std_types::RResult<$crate::RPlugin, $crate::RError> {
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
    pub new: extern "C" fn(RVec<RString>) -> RResult<RPlugin, RError>,
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
        Self(RBoxPluginTrait::from_ptr(RBox::new(plugin), TU_Opaque))
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
}

/// FFI-safe equivalent of [`Plugin`] trait.
#[sabi_trait]
trait RPluginTrait: 'static {
    fn name(&self) -> RString;
    fn joint_trajectory_client(&self) -> ROption<RJointTrajectoryClient>;
    fn speaker(&self) -> ROption<RSpeaker>;
    fn move_base(&self) -> ROption<RMoveBase>;
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
}

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

/// FFI-safe equivalent of [`Arc<dyn arci::JointTrajectoryClient>`](arci::JointTrajectoryClient).
#[repr(C)]
#[derive(StableAbi)]
pub struct RJointTrajectoryClient(RBoxJointTrajectoryClientTrait);

impl RJointTrajectoryClient {
    pub fn new<C>(c: Arc<C>) -> Self
    where
        C: ?Sized + StaticJointTrajectoryClient,
    {
        Self(RBoxJointTrajectoryClientTrait::from_ptr(
            RBox::new(c),
            TU_Opaque,
        ))
    }
}

impl Clone for RJointTrajectoryClient {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl StaticJointTrajectoryClient for RJointTrajectoryClient {
    fn joint_names(&self) -> Vec<String> {
        self.0.joint_names().into_iter().map(|s| s.into()).collect()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        match self.0.current_joint_positions() {
            ROk(p) => Ok(p.into_iter().map(f64::from).collect()),
            RErr(e) => Err(format_err!("{}", e).into()),
        }
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture<'static>, arci::Error> {
        match self.0.send_joint_positions(
            positions.into_iter().map(Rf64::from).collect(),
            duration.into(),
        ) {
            ROk(wait) => Ok(wait.into()),
            RErr(e) => Err(format_err!("{}", e).into()),
        }
    }

    fn send_joint_trajectory(
        &self,
        trajectory: Vec<arci::TrajectoryPoint>,
    ) -> Result<WaitFuture<'static>, arci::Error> {
        match self
            .0
            .send_joint_trajectory(trajectory.into_iter().map(RTrajectoryPoint::from).collect())
        {
            ROk(wait) => Ok(wait.into()),
            RErr(e) => Err(format_err!("{}", e).into()),
        }
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

/// FFI-safe equivalent of [`StaticJointTrajectoryClient`] trait.
#[sabi_trait]
trait RJointTrajectoryClientTrait: Send + Sync + Clone + 'static {
    fn joint_names(&self) -> RVec<RString>;
    fn current_joint_positions(&self) -> RResult<RVec<Rf64>, RError>;
    fn send_joint_positions(
        &self,
        positions: RVec<Rf64>,
        duration: RDuration,
    ) -> RResult<RBlockingWait, RError>;
    fn send_joint_trajectory(
        &self,
        trajectory: RVec<RTrajectoryPoint>,
    ) -> RResult<RBlockingWait, RError>;
}

type RBoxJointTrajectoryClientTrait = RJointTrajectoryClientTrait_TO<RBox<()>>;

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

    fn current_joint_positions(&self) -> RResult<RVec<Rf64>, RError> {
        match StaticJointTrajectoryClient::current_joint_positions(&**self) {
            Ok(p) => ROk(p.into_iter().map(Rf64::from).collect()),
            Err(e) => RErr(e.into()),
        }
    }

    fn send_joint_positions(
        &self,
        positions: RVec<Rf64>,
        duration: RDuration,
    ) -> RResult<RBlockingWait, RError> {
        match StaticJointTrajectoryClient::send_joint_positions(
            &**self,
            positions.into_iter().map(f64::from).collect(),
            duration.into(),
        ) {
            Ok(wait) => ROk(wait.into()),
            Err(e) => RErr(e.into()),
        }
    }

    fn send_joint_trajectory(
        &self,
        trajectory: RVec<RTrajectoryPoint>,
    ) -> RResult<RBlockingWait, RError> {
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

#[repr(C)]
#[derive(StableAbi)]
#[must_use]
struct RBlockingWait(RBoxWait);

impl RBlockingWait {
    fn from_fn(f: impl FnOnce() -> RResult<(), RError> + Send + 'static) -> Self {
        Self(RBoxWait::from_value(
            f,
            abi_stable::type_level::unerasability::TU_Opaque,
        ))
    }
}

impl From<WaitFuture<'static>> for RBlockingWait {
    fn from(wait: WaitFuture<'static>) -> Self {
        Self::from_fn(move || {
            // TODO: use new_multi_thread?
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(async move { wait.await.map_err(RError::from).into() })
        })
    }
}

impl From<RBlockingWait> for WaitFuture<'static> {
    fn from(wait: RBlockingWait) -> Self {
        // Creates a WaitFuture that waits until Wait::wait done only if the future
        // is polled. This future is a bit tricky, but it's more efficient than
        // using only `tokio::task::spawn_blocking` because it doesn't spawn a thread
        // if the WaitFuture is ignored.
        WaitFuture::new(async move {
            tokio::task::spawn_blocking(move || wait.0.wait())
                .await
                .map_err(|e| arci::Error::Other(e.into()))?
                .into_result()
                .map_err(|e| arci::Error::Other(format_err!("{}", e)))?;
            Ok(())
        })
    }
}

#[sabi_trait]
trait RWaitTrait: Send + 'static {
    fn wait(self) -> RResult<(), RError>;
}

type RBoxWait = RWaitTrait_TO<RBox<()>>;

impl<F> RWaitTrait for F
where
    F: FnOnce() -> RResult<(), RError> + Send + 'static,
{
    fn wait(self) -> RResult<(), RError> {
        self()
    }
}

/// FFI-safe equivalent of [`f64`].
#[repr(C)]
#[derive(Debug, Clone, Copy, StableAbi)]
struct Rf64 {
    mantissa: u64,
    exponent: i16,
    sign: i8,
}

impl From<f64> for Rf64 {
    fn from(val: f64) -> Self {
        let (mantissa, exponent, sign) = Float::integer_decode(val);
        Self {
            mantissa,
            exponent,
            sign,
        }
    }
}

impl From<Rf64> for f64 {
    fn from(val: Rf64) -> Self {
        // https://docs.rs/num-traits/0.2/num_traits/float/trait.Float.html#tymethod.integer_decode
        let Rf64 {
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

/// FFI-safe equivalent of [`arci::TrajectoryPoint`].
#[repr(C)]
#[derive(StableAbi, Clone, Debug)]
struct RTrajectoryPoint {
    positions: RVec<Rf64>,
    velocities: ROption<RVec<Rf64>>,
    time_from_start: RDuration,
}

impl From<arci::TrajectoryPoint> for RTrajectoryPoint {
    fn from(val: arci::TrajectoryPoint) -> Self {
        Self {
            positions: val.positions.into_iter().map(Rf64::from).collect(),
            velocities: val
                .velocities
                .map(|v| v.into_iter().map(Rf64::from).collect())
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

/// FFI-safe equivalent of [`Arc<dyn arci::Speaker>`](arci::Speaker).
#[repr(C)]
#[derive(StableAbi)]
pub struct RSpeaker(RBoxSpeakerTrait);

impl RSpeaker {
    pub fn new<T>(c: Arc<T>) -> Self
    where
        T: ?Sized + arci::Speaker + 'static,
    {
        Self(RBoxSpeakerTrait::from_ptr(RBox::new(c), TU_Opaque))
    }
}

impl Clone for RSpeaker {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl arci::Speaker for RSpeaker {
    fn speak(&self, message: &str) -> Result<WaitFuture<'static>, arci::Error> {
        match self.0.speak(message.into()) {
            ROk(wait) => Ok(wait.into()),
            RErr(e) => Err(e.into()),
        }
    }
}

/// FFI-safe equivalent of [`arci::Speaker`] trait.
#[sabi_trait]
trait RSpeakerTrait: Send + Sync + Clone + 'static {
    fn speak(&self, message: RStr<'_>) -> RResult<RBlockingWait, RError>;
}

type RBoxSpeakerTrait = RSpeakerTrait_TO<RBox<()>>;

impl<T> RSpeakerTrait for Arc<T>
where
    T: ?Sized + arci::Speaker + 'static,
{
    fn speak(&self, message: RStr<'_>) -> RResult<RBlockingWait, RError> {
        match arci::Speaker::speak(&**self, message.into()) {
            Ok(wait) => ROk(wait.into()),
            Err(e) => RErr(e.into()),
        }
    }
}

/// FFI-safe equivalent of [`Arc<dyn arci::MoveBase>`](arci::MoveBase).
#[repr(C)]
#[derive(StableAbi)]
pub struct RMoveBase(RBoxMoveBaseTrait);

impl RMoveBase {
    pub fn new<T>(c: Arc<T>) -> Self
    where
        T: ?Sized + arci::MoveBase + 'static,
    {
        Self(RBoxMoveBaseTrait::from_ptr(RBox::new(c), TU_Opaque))
    }
}

impl Clone for RMoveBase {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl arci::MoveBase for RMoveBase {
    fn send_velocity(&self, velocity: &arci::BaseVelocity) -> Result<(), arci::Error> {
        match self.0.send_velocity(&RBaseVelocity::from(*velocity)) {
            ROk(()) => Ok(()),
            RErr(e) => Err(e.into()),
        }
    }

    fn current_velocity(&self) -> Result<arci::BaseVelocity, arci::Error> {
        match self.0.current_velocity() {
            ROk(v) => Ok(v.into()),
            RErr(e) => Err(e.into()),
        }
    }
}

/// FFI-safe equivalent of [`arci::MoveBase`] trait.
#[sabi_trait]
trait RMoveBaseTrait: Send + Sync + Clone + 'static {
    fn send_velocity(&self, velocity: &RBaseVelocity) -> RResult<(), RError>;
    fn current_velocity(&self) -> RResult<RBaseVelocity, RError>;
}

type RBoxMoveBaseTrait = RMoveBaseTrait_TO<RBox<()>>;

impl<T> RMoveBaseTrait for Arc<T>
where
    T: ?Sized + arci::MoveBase + 'static,
{
    fn send_velocity(&self, velocity: &RBaseVelocity) -> RResult<(), RError> {
        match arci::MoveBase::send_velocity(&**self, &arci::BaseVelocity::from(*velocity)) {
            Ok(()) => ROk(()),
            Err(e) => RErr(e.into()),
        }
    }

    fn current_velocity(&self) -> RResult<RBaseVelocity, RError> {
        match arci::MoveBase::current_velocity(&**self) {
            Ok(v) => ROk(v.into()),
            Err(e) => RErr(e.into()),
        }
    }
}

/// FFI-safe equivalent of [`arci::BaseVelocity`].
#[repr(C)]
#[derive(StableAbi, Clone, Copy, Debug)]
struct RBaseVelocity {
    x: Rf64,
    y: Rf64,
    theta: Rf64,
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
