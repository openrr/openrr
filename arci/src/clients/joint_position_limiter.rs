use std::{f64, ops::RangeInclusive};

use schemars::{gen::SchemaGenerator, schema::Schema, JsonSchema};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use tracing::debug;
use urdf_rs::JointType;

use crate::{
    error::Error,
    traits::{JointTrajectoryClient, TrajectoryPoint},
    waits::WaitFuture,
};

pub struct JointPositionLimiter<C>
where
    C: JointTrajectoryClient,
{
    client: C,
    limits: Vec<JointPositionLimit>,
    strategy: JointPositionLimiterStrategy,
}

impl<C> JointPositionLimiter<C>
where
    C: JointTrajectoryClient,
{
    /// Creates a new `JointPositionLimiter` with the given position limits.
    ///
    /// # Panics
    ///
    /// Panics if the lengths of `limits` and joints that `client` handles are different.
    pub fn new(client: C, limits: Vec<JointPositionLimit>) -> Self {
        Self::new_with_strategy(client, limits, Default::default())
    }

    pub fn new_with_strategy(
        client: C,
        limits: Vec<JointPositionLimit>,
        strategy: JointPositionLimiterStrategy,
    ) -> Self {
        assert!(client.joint_names().len() == limits.len());
        Self {
            client,
            limits,
            strategy,
        }
    }

    /// Creates a new `JointPositionLimiter` with the position limits defined in URDF.
    pub fn from_urdf(client: C, joints: &[urdf_rs::Joint]) -> Result<Self, Error> {
        Self::from_urdf_with_strategy(client, joints, Default::default())
    }

    pub fn from_urdf_with_strategy(
        client: C,
        joints: &[urdf_rs::Joint],
        strategy: JointPositionLimiterStrategy,
    ) -> Result<Self, Error> {
        let mut limits = Vec::new();
        for joint_name in client.joint_names() {
            if let Some(i) = joints.iter().position(|j| j.name == *joint_name) {
                let joint = &joints[i];
                let limit = if JointType::Continuous == joint.joint_type {
                    // Continuous joint has no limit.
                    JointPositionLimit::none()
                } else {
                    (joint.limit.lower..=joint.limit.upper).into()
                };
                limits.push(limit);
            } else {
                return Err(Error::NoJoint(joint_name.into()));
            }
        }

        Ok(Self {
            client,
            limits,
            strategy,
        })
    }

    pub fn set_strategy(&mut self, strategy: JointPositionLimiterStrategy) {
        self.strategy = strategy;
    }

    fn check_joint_position(&self, positions: &mut Vec<f64>) -> Result<(), Error> {
        assert_eq!(positions.len(), self.limits.len());
        for (i, limit, position) in self
            .limits
            .iter()
            .zip(positions)
            .enumerate()
            .filter_map(|(i, (l, p))| l.range().map(|l| (i, l, p)))
        {
            if limit.contains(&position) {
                continue;
            }
            match self.strategy {
                JointPositionLimiterStrategy::Error => {
                    return Err(Error::OutOfLimit {
                        name: self.client.joint_names()[i].clone(),
                        position: *position,
                        limit,
                    });
                }
                JointPositionLimiterStrategy::Clamp => {
                    debug!(
                        "Out of limit: joint={}, position={}, limit={:?}",
                        self.client.joint_names()[i],
                        position,
                        limit,
                    );
                    *position = position.clamp(*limit.start(), *limit.end());
                }
            }
        }
        Ok(())
    }
}

impl<C> JointTrajectoryClient for JointPositionLimiter<C>
where
    C: JointTrajectoryClient,
{
    fn joint_names(&self) -> &[String] {
        self.client.joint_names()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        let mut positions = self.client.current_joint_positions()?;
        self.check_joint_position(&mut positions)?;
        Ok(positions)
    }

    fn send_joint_positions(
        &self,
        mut positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        self.check_joint_position(&mut positions)?;
        self.client.send_joint_positions(positions, duration)
    }

    fn send_joint_trajectory(
        &self,
        mut trajectory: Vec<TrajectoryPoint>,
    ) -> Result<WaitFuture, Error> {
        for tp in &mut trajectory {
            self.check_joint_position(&mut tp.positions)?;
        }
        self.client.send_joint_trajectory(trajectory)
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct JointPositionLimit(Option<JointPositionLimitInner>);

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct JointPositionLimitInner {
    lower: f64,
    upper: f64,
}

impl JointPositionLimit {
    pub fn new(lower: f64, upper: f64) -> Self {
        Self(Some(JointPositionLimitInner { lower, upper }))
    }

    pub fn none() -> Self {
        Self::default()
    }

    pub fn is_none(&self) -> bool {
        self.0.is_none()
    }

    pub fn range(&self) -> Option<RangeInclusive<f64>> {
        self.0.map(|l| l.lower..=l.upper)
    }

    pub fn lower(&self) -> Option<f64> {
        self.0.map(|l| l.lower)
    }

    pub fn upper(&self) -> Option<f64> {
        self.0.map(|l| l.upper)
    }
}

impl From<RangeInclusive<f64>> for JointPositionLimit {
    fn from(r: RangeInclusive<f64>) -> Self {
        Self::new(*r.start(), *r.end())
    }
}

impl From<urdf_rs::JointLimit> for JointPositionLimit {
    fn from(l: urdf_rs::JointLimit) -> Self {
        Self::new(l.lower, l.upper)
    }
}

impl<'de> Deserialize<'de> for JointPositionLimit {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(untagged, deny_unknown_fields)]
        enum De {
            Limit(JointPositionLimitInner),
            Empty {},
        }
        Ok(match De::deserialize(deserializer)? {
            De::Empty {} => Self::none(),
            De::Limit(l) => Self(Some(l)),
        })
    }
}

impl Serialize for JointPositionLimit {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        #[derive(Serialize)]
        #[serde(untagged)]
        enum Ser {
            Limit(JointPositionLimitInner),
            Empty {},
        }

        match self.0 {
            None => Ser::Empty {}.serialize(serializer),
            Some(l) => Ser::Limit(l).serialize(serializer),
        }
    }
}

impl JsonSchema for JointPositionLimit {
    fn schema_name() -> String {
        "JointPositionLimit".into()
    }

    fn json_schema(gen: &mut SchemaGenerator) -> Schema {
        // As workaround for https://github.com/tamasfe/taplo/issues/57,
        // use struct with option value instead of enum.
        #[allow(dead_code)]
        #[derive(JsonSchema)]
        struct JointPositionLimitRepr {
            lower: Option<f64>,
            upper: Option<f64>,
        }

        JointPositionLimitRepr::json_schema(gen)
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum JointPositionLimiterStrategy {
    /// If the position is out of the limit, handle it as the same value as the limit.
    Clamp,
    /// If the position is out of the limit, return an error.
    Error,
}

impl Default for JointPositionLimiterStrategy {
    fn default() -> Self {
        Self::Clamp
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use assert_approx_eq::assert_approx_eq;

    use super::*;
    use crate::DummyJointTrajectoryClient;

    const SECOND: Duration = Duration::from_secs(1);

    #[test]
    #[should_panic]
    fn mismatch_size() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        JointPositionLimiter::new(client, vec![(1.0..=2.0).into(), (2.0..=3.0).into()]);
    }

    #[test]
    fn joint_names() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned(), "b".to_owned()]);
        let limiter =
            JointPositionLimiter::new(client, vec![(1.0..=2.0).into(), (2.0..=3.0).into()]);
        let joint_names = limiter.joint_names();
        assert_eq!(joint_names.len(), 2);
        assert_eq!(joint_names[0], "a");
        assert_eq!(joint_names[1], "b");
    }

    #[tokio::test]
    async fn send_joint_positions_none_limited() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let mut client = JointPositionLimiter::new(client, vec![(1.0..=2.0).into()]);

        for &strategy in &[
            JointPositionLimiterStrategy::Clamp,
            JointPositionLimiterStrategy::Error,
        ] {
            client.set_strategy(strategy);

            client
                .send_joint_positions(vec![1.0], SECOND)
                .unwrap()
                .await
                .unwrap();
            assert_approx_eq!(client.current_joint_positions().unwrap()[0], 1.0);

            client
                .send_joint_positions(vec![2.0], SECOND)
                .unwrap()
                .await
                .unwrap();
            assert_approx_eq!(client.current_joint_positions().unwrap()[0], 2.0);
        }
    }

    #[tokio::test]
    async fn send_joint_positions_limited_rounded() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let client = JointPositionLimiter::new(client, vec![(1.0..=2.0).into()]);

        client
            .send_joint_positions(vec![0.0], SECOND)
            .unwrap()
            .await
            .unwrap();
        assert_approx_eq!(client.current_joint_positions().unwrap()[0], 1.0);

        client
            .send_joint_positions(vec![3.0], SECOND)
            .unwrap()
            .await
            .unwrap();
        assert_approx_eq!(client.current_joint_positions().unwrap()[0], 2.0);
    }

    #[tokio::test]
    async fn send_joint_positions_limited_error() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let client = JointPositionLimiter::new_with_strategy(
            client,
            vec![(1.0..=2.0).into()],
            JointPositionLimiterStrategy::Error,
        );

        let e = client
            .send_joint_positions(vec![0.0], SECOND)
            .err()
            .unwrap();
        assert_error(e, 0.0);

        let e = client
            .send_joint_positions(vec![3.0], SECOND)
            .err()
            .unwrap();
        assert_error(e, 3.0);
    }

    #[tokio::test]
    async fn send_joint_trajectory_none_limited() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let mut client = JointPositionLimiter::new(client, vec![(1.0..=2.0).into()]);

        for &strategy in &[
            JointPositionLimiterStrategy::Clamp,
            JointPositionLimiterStrategy::Error,
        ] {
            client.set_strategy(strategy);

            client
                .send_joint_trajectory(vec![
                    TrajectoryPoint::new(vec![1.0], SECOND * 2),
                    TrajectoryPoint::new(vec![1.5], SECOND * 3),
                ])
                .unwrap()
                .await
                .unwrap();
            assert_approx_eq!(client.current_joint_positions().unwrap()[0], 1.5);

            client
                .send_joint_trajectory(vec![
                    TrajectoryPoint::new(vec![1.7], SECOND * 2),
                    TrajectoryPoint::new(vec![2.0], SECOND * 3),
                ])
                .unwrap()
                .await
                .unwrap();
            assert_approx_eq!(client.current_joint_positions().unwrap()[0], 2.0);
        }
    }

    #[tokio::test]
    async fn send_joint_trajectory_limited_rounded() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let client = JointPositionLimiter::new(client, vec![(1.0..=2.0).into()]);

        client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![0.0], SECOND * 2),
                TrajectoryPoint::new(vec![0.5], SECOND * 3),
            ])
            .unwrap()
            .await
            .unwrap();
        assert_approx_eq!(client.current_joint_positions().unwrap()[0], 1.0);

        client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![2.5], SECOND * 2),
                TrajectoryPoint::new(vec![3.0], SECOND * 3),
            ])
            .unwrap()
            .await
            .unwrap();
        assert_approx_eq!(client.current_joint_positions().unwrap()[0], 2.0);
    }

    #[tokio::test]
    async fn send_joint_trajectory_limited_error() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let client = JointPositionLimiter::new_with_strategy(
            client,
            vec![(1.0..=2.0).into()],
            JointPositionLimiterStrategy::Error,
        );

        let e = client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![0.0], SECOND * 2),
                TrajectoryPoint::new(vec![0.5], SECOND * 3),
            ])
            .err()
            .unwrap();
        assert_error(e, 0.0);

        let e = client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![1.0], SECOND * 2),
                TrajectoryPoint::new(vec![0.5], SECOND * 3),
            ])
            .err()
            .unwrap();
        assert_error(e, 0.5);

        let e = client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![2.5], SECOND * 2),
                TrajectoryPoint::new(vec![3.0], SECOND * 3),
            ])
            .err()
            .unwrap();
        assert_error(e, 2.5);

        let e = client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![2.0], SECOND * 2),
                TrajectoryPoint::new(vec![3.0], SECOND * 3),
            ])
            .err()
            .unwrap();
        assert_error(e, 3.0);
    }

    fn assert_error(e: Error, position: f64) {
        match e {
            Error::OutOfLimit { position: p, .. } => assert_approx_eq!(p, position),
            _ => panic!("{:?}", e),
        }
    }

    #[test]
    fn from_urdf() {
        let s = r##"
            <robot name="robo">
                <joint name="a" type="revolute">
                    <origin xyz="0.0 0.0 0.0" />
                    <parent link="b" />
                    <child link="c" />
                    <axis xyz="0 1 0" />
                    <limit lower="-2" upper="1.0" effort="0" velocity="1.0"/>
                </joint>
            </robot>
        "##;
        let urdf_robot = urdf_rs::read_from_string(s).unwrap();
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let limiter = JointPositionLimiter::from_urdf(client, &urdf_robot.joints).unwrap();
        assert_approx_eq!(limiter.limits[0].lower().unwrap(), -2.0);
        assert_approx_eq!(limiter.limits[0].upper().unwrap(), 1.0);

        // joint name mismatch
        let urdf_robot = urdf_rs::read_from_string(s).unwrap();
        let client = DummyJointTrajectoryClient::new(vec!["unknown".to_owned()]);
        let e = JointPositionLimiter::from_urdf(client, &urdf_robot.joints)
            .err()
            .unwrap();
        assert!(matches!(e, Error::NoJoint(..)));
    }

    #[test]
    fn serde_joint_position_limit() {
        #[derive(Serialize, Deserialize)]
        struct T {
            limits: Vec<JointPositionLimit>,
        }

        let l: T = toml::from_str("limits = [{ lower = 0.0, upper = 1.0 }]").unwrap();
        assert_approx_eq!(l.limits[0].lower().unwrap(), 0.0);
        assert_approx_eq!(l.limits[0].upper().unwrap(), 1.0);

        let l: T = toml::from_str("limits = [{}]").unwrap();
        assert!(l.limits[0].is_none());

        let l: T = toml::from_str(
            "limits = [\
                { lower = 0.0, upper = 1.0 },\
                {},\
                { lower = 1.0, upper = 2.0 }\
            ]",
        )
        .unwrap();
        assert_approx_eq!(l.limits[0].lower().unwrap(), 0.0);
        assert_approx_eq!(l.limits[0].upper().unwrap(), 1.0);
        assert!(l.limits[1].is_none());
        assert_approx_eq!(l.limits[2].lower().unwrap(), 1.0);
        assert_approx_eq!(l.limits[2].upper().unwrap(), 2.0);

        // TODO: We want to serialize to inline table: https://github.com/alexcrichton/toml-rs/issues/265
        assert_eq!(
            toml::to_string(&l).unwrap(),
            "[[limits]]\n\
             lower = 0.0\n\
             upper = 1.0\n\
             \n\
             [[limits]]\n\
             \n\
             [[limits]]\n\
             lower = 1.0\n\
             upper = 2.0\n\
            "
        );
    }
}
