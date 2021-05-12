//! Experimental plugin support for [`arci`].

#![warn(rust_2018_idioms)]

use std::{fmt, future::Future, path::Path, sync::Arc, time::Duration};

use abi_stable::{
    declare_root_module_statics,
    erased_types::TU_Opaque,
    library::{lib_header_from_path, RootModule},
    package_version_strings, sabi_trait,
    sabi_types::VersionStrings,
    std_types::{RBox, RBoxError, RDuration, RErr, ROk, ROption, RResult, RString, RVec},
    StableAbi,
};
use anyhow::{format_err, Result};
#[doc(no_inline)]
pub use arci::{Error, TrajectoryPoint, WaitFuture};
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

/// Ffi-safe equivalent of [`Box<dyn Plugin>`](Plugin).
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
}

pub trait Plugin: 'static {
    fn name(&self) -> String;
    fn joint_trajectory_client(&self) -> Option<Arc<dyn StaticJointTrajectoryClient>> {
        None
    }
}

/// Ffi-safe equivalent of [`Plugin`] trait.
#[sabi_trait]
trait RPluginTrait: 'static {
    fn name(&self) -> RString;
    fn joint_trajectory_client(&self) -> ROption<RJointTrajectoryClient>;
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
}

/// Ffi-safe equivalent of [`arci::Error`].
#[repr(C)]
#[derive(StableAbi)]
pub struct RError {
    repr: RBoxError,
}

impl From<Error> for RError {
    fn from(e: Error) -> Self {
        Self {
            // TODO: propagate error kind.
            repr: RBoxError::from_box(Box::new(e)),
        }
    }
}

impl From<RError> for Error {
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

/// Ffi-safe equivalent of [`Arc<dyn arci::JointTrajectoryClient>`](arci::JointTrajectoryClient).
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
        trajectory: Vec<TrajectoryPoint>,
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
        trajectory: Vec<TrajectoryPoint>,
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
        duration: std::time::Duration,
    ) -> Result<WaitFuture<'static>, arci::Error>;
    fn send_joint_trajectory(
        &self,
        trajectory: Vec<TrajectoryPoint>,
    ) -> Result<WaitFuture<'static>, arci::Error>;
}

/// Ffi-safe equivalent of [`StaticJointTrajectoryClient`] trait.
#[sabi_trait]
trait RJointTrajectoryClientTrait: Send + Sync + Clone + 'static {
    fn joint_names(&self) -> RVec<RString>;
    fn current_joint_positions(&self) -> RResult<RVec<Rf64>, RBoxError>;
    fn send_joint_positions(
        &self,
        positions: RVec<Rf64>,
        duration: RDuration,
    ) -> RResult<RBlockingWait, RBoxError>;
    fn send_joint_trajectory(
        &self,
        trajectory: RVec<RTrajectoryPoint>,
    ) -> RResult<RBlockingWait, RBoxError>;
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

    fn current_joint_positions(&self) -> RResult<RVec<Rf64>, RBoxError> {
        match StaticJointTrajectoryClient::current_joint_positions(&**self) {
            Ok(p) => ROk(p.into_iter().map(Rf64::from).collect()),
            Err(e) => RErr(RBoxError::from_box(e.into())),
        }
    }

    fn send_joint_positions(
        &self,
        positions: RVec<Rf64>,
        duration: RDuration,
    ) -> RResult<RBlockingWait, RBoxError> {
        match StaticJointTrajectoryClient::send_joint_positions(
            &**self,
            positions.into_iter().map(f64::from).collect(),
            duration.into(),
        ) {
            Ok(wait) => ROk(RBlockingWait::from_async(async move {
                wait.await.map_err(|e| RBoxError::from_box(e.into())).into()
            })),
            Err(e) => RErr(RBoxError::from_box(e.into())),
        }
    }

    fn send_joint_trajectory(
        &self,
        trajectory: RVec<RTrajectoryPoint>,
    ) -> RResult<RBlockingWait, RBoxError> {
        match StaticJointTrajectoryClient::send_joint_trajectory(
            &**self,
            trajectory.into_iter().map(TrajectoryPoint::from).collect(),
        ) {
            Ok(wait) => ROk(RBlockingWait::from_async(async move {
                wait.await.map_err(|e| RBoxError::from_box(e.into())).into()
            })),
            Err(e) => RErr(RBoxError::from_box(e.into())),
        }
    }
}

#[repr(C)]
#[derive(StableAbi)]
#[must_use]
struct RBlockingWait(RBoxWait);

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

impl RBlockingWait {
    fn from_fn(f: impl FnOnce() -> RResult<(), RBoxError> + Send + 'static) -> Self {
        Self(RBoxWait::from_value(
            f,
            abi_stable::type_level::unerasability::TU_Opaque,
        ))
    }

    fn from_async(f: impl Future<Output = RResult<(), RBoxError>> + Send + 'static) -> Self {
        Self::from_fn(move || {
            // TODO: use new_multi_thread?
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(f)
        })
    }
}

type RBoxWait = RWaitTrait_TO<RBox<()>>;

#[sabi_trait]
trait RWaitTrait: Send + 'static {
    fn wait(self) -> RResult<(), RBoxError>;
}

impl<F> RWaitTrait for F
where
    F: FnOnce() -> RResult<(), RBoxError> + Send + 'static,
{
    fn wait(self) -> RResult<(), RBoxError> {
        self()
    }
}

/// Ffi-safe equivalent of [`f64`].
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

/// Ffi-safe equivalent of [`arci::TrajectoryPoint`].
#[repr(C)]
#[derive(StableAbi, Clone, Debug)]
struct RTrajectoryPoint {
    positions: RVec<Rf64>,
    velocities: ROption<RVec<Rf64>>,
    time_from_start: RDuration,
}

impl From<arci::TrajectoryPoint> for RTrajectoryPoint {
    fn from(p: arci::TrajectoryPoint) -> Self {
        Self {
            positions: p.positions.into_iter().map(Rf64::from).collect(),
            velocities: p
                .velocities
                .map(|v| v.into_iter().map(Rf64::from).collect())
                .into(),
            time_from_start: p.time_from_start.into(),
        }
    }
}

impl From<RTrajectoryPoint> for arci::TrajectoryPoint {
    fn from(val: RTrajectoryPoint) -> Self {
        arci::TrajectoryPoint {
            positions: val.positions.into_iter().map(f64::from).collect(),
            velocities: val
                .velocities
                .into_option()
                .map(|v| v.into_iter().map(f64::from).collect()),
            time_from_start: val.time_from_start.into(),
        }
    }
}
