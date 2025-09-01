//! Deserialize tracing log in JSON format

use std::time::{Duration, SystemTime};

use arci::{BaseVelocity, nalgebra};
use serde::Deserialize;

pub type Timestamp = chrono::DateTime<chrono::Utc>;

pub fn from_str(lines: &str) -> Result<Vec<TracingLog>, arci::Error> {
    let mut res = vec![];
    for line in lines.lines() {
        let value: serde_json::Value =
            serde_json::from_str(line).map_err(|e| arci::Error::Other(e.into()))?;
        // ignore unrelated line and log for other target (library/binary/module).
        if !matches!(value.get("target"), Some(target) if target == "openrr_tracing") {
            continue;
        }
        let Some(fields) = value.get("fields") else {
            continue;
        };
        match fields.get("method") {
            Some(v) if v == "arci::Localization::current_pose" => {
                let log: CurrentPoseLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::CurrentPose(log));
            }
            Some(v) if v == "arci::MotorDriveEffort::set_motor_effort" => {
                let log: SetMotorEffortLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::SetMotorEffort(log));
            }
            Some(v) if v == "arci::MotorDriveEffort::get_motor_effort" => {
                let log: GetMotorEffortLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::GetMotorEffort(log));
            }
            Some(v) if v == "arci::MotorDrivePosition::set_motor_position" => {
                let log: SetMotorPositionLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::SetMotorPosition(log));
            }
            Some(v) if v == "arci::MotorDrivePosition::get_motor_position" => {
                let log: GetMotorPositionLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::GetMotorPosition(log));
            }
            Some(v) if v == "arci::MotorDriveVelocity::set_motor_velocity" => {
                let log: SetMotorVelocityLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::SetMotorVelocity(log));
            }
            Some(v) if v == "arci::MotorDriveVelocity::get_motor_velocity" => {
                let log: GetMotorVelocityLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::GetMotorVelocity(log));
            }
            Some(v) if v == "arci::MoveBase::send_velocity" => {
                let log: SendVelocityLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::SendVelocity(log));
            }
            Some(v) if v == "arci::MoveBase::current_velocity" => {
                let log: CurrentVelocityLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::CurrentVelocity(log));
            }
            Some(v) if v == "arci::Navigation::send_goal_pose" => {
                let log: SendGoalPoseLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::SendGoalPose(log));
            }
            Some(v) if v == "arci::Navigation::cancel" => {
                let log: NavigationCancelLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::NavigationCancel(log));
            }
            Some(v) if v == "arci::Speaker::speak" => {
                let log: SpeakLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::Speak(log));
            }
            Some(v) if v == "arci::TransformResolver::resolve_transformation" => {
                let log: ResolveTransformationLog =
                    serde_json::from_value(value).map_err(|e| arci::Error::Other(e.into()))?;
                res.push(TracingLog::ResolveTransformation(log));
            }
            _ => continue, // TODO
        }
    }
    Ok(res)
}

#[derive(Debug)]
#[non_exhaustive]
pub enum TracingLog {
    /// [`arci::Localization::current_pose`]
    CurrentPose(CurrentPoseLog),

    /// [`arci::MotorDriveEffort::set_motor_effort`]
    SetMotorEffort(SetMotorEffortLog),
    /// [`arci::MotorDriveEffort::get_motor_effort`]
    GetMotorEffort(GetMotorEffortLog),

    /// [`arci::MotorDrivePosition::set_motor_position`]
    SetMotorPosition(SetMotorPositionLog),
    /// [`arci::MotorDrivePosition::get_motor_position`]
    GetMotorPosition(GetMotorPositionLog),

    /// [`arci::MotorDriveVelocity::set_motor_velocity`]
    SetMotorVelocity(SetMotorVelocityLog),
    /// [`arci::MotorDriveVelocity::get_motor_velocity`]
    GetMotorVelocity(GetMotorVelocityLog),

    /// [`arci::MoveBase::send_velocity`]
    SendVelocity(SendVelocityLog),
    /// [`arci::MoveBase::current_velocity`]
    CurrentVelocity(CurrentVelocityLog),

    /// [`arci::Navigation::send_goal_pose`]
    SendGoalPose(SendGoalPoseLog),
    /// [`arci::Navigation::cancel`]
    NavigationCancel(NavigationCancelLog),

    /// [`arci::Speaker::speak`]
    Speak(SpeakLog),

    /// [`arci::TransformResolver::resolve_transformation`]
    ResolveTransformation(ResolveTransformationLog),
}

#[derive(Deserialize)]
struct RawTracingLog<Fields> {
    timestamp: Timestamp,
    fields: Fields,
}

// =============================================================================
// arci::Localization

#[derive(Debug)]
#[non_exhaustive]
pub struct CurrentPoseLog {
    pub timestamp: Timestamp,
    pub frame_id: String,
    pub pose: arci::Isometry2<f64>,
}

impl<'de> Deserialize<'de> for CurrentPoseLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Fields {
            frame_id: String,
            pose_rotation_re: f64,
            pose_rotation_im: f64,
            pose_translation_x: f64,
            pose_translation_y: f64,
        }
        let v = RawTracingLog::<Fields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            frame_id: v.fields.frame_id,
            pose: arci::Isometry2::from_parts(
                nalgebra::Translation2::new(
                    v.fields.pose_translation_x,
                    v.fields.pose_translation_y,
                ),
                nalgebra::UnitComplex::from_complex(nalgebra::Complex {
                    re: v.fields.pose_rotation_re,
                    im: v.fields.pose_rotation_im,
                }),
            ),
        })
    }
}

// =============================================================================
// arci::MotorDriveEffort

// Currently, both SetMotorEffortLog and GetMotorEffortLog are the same.
// However, we may want to add result field to SetMotorEffortLog.
#[derive(Debug)]
#[non_exhaustive]
pub struct SetMotorEffortLog {
    pub timestamp: Timestamp,
    pub effort: f64,
}
#[derive(Debug)]
#[non_exhaustive]
pub struct GetMotorEffortLog {
    pub timestamp: Timestamp,
    pub effort: f64,
}

#[derive(Deserialize)]
struct MotorEffortLogFields {
    effort: f64,
}
impl<'de> Deserialize<'de> for SetMotorEffortLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let v = RawTracingLog::<MotorEffortLogFields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            effort: v.fields.effort,
        })
    }
}
impl<'de> Deserialize<'de> for GetMotorEffortLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let v = RawTracingLog::<MotorEffortLogFields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            effort: v.fields.effort,
        })
    }
}

// =============================================================================
// arci::MotorDrivePosition

// Currently, both SetMotorPositionLog and GetMotorPositionLog are the same.
// However, we may want to add result field to SetMotorPositionLog.
#[derive(Debug)]
#[non_exhaustive]
pub struct SetMotorPositionLog {
    pub timestamp: Timestamp,
    pub position: f64,
}
#[derive(Debug)]
#[non_exhaustive]
pub struct GetMotorPositionLog {
    pub timestamp: Timestamp,
    pub position: f64,
}

#[derive(Deserialize)]
struct MotorPositionLogFields {
    position: f64,
}
impl<'de> Deserialize<'de> for SetMotorPositionLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let v = RawTracingLog::<MotorPositionLogFields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            position: v.fields.position,
        })
    }
}
impl<'de> Deserialize<'de> for GetMotorPositionLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let v = RawTracingLog::<MotorPositionLogFields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            position: v.fields.position,
        })
    }
}

// =============================================================================
// arci::MotorDriveVelocity

// Currently, both SetMotorVelocityLog and GetMotorVelocityLog are the same.
// However, we may want to add result field to SetMotorVelocityLog.
#[derive(Debug)]
#[non_exhaustive]
pub struct SetMotorVelocityLog {
    pub timestamp: Timestamp,
    pub velocity: f64,
}
#[derive(Debug)]
#[non_exhaustive]
pub struct GetMotorVelocityLog {
    pub timestamp: Timestamp,
    pub velocity: f64,
}

#[derive(Deserialize)]
struct MotorVelocityLogFields {
    velocity: f64,
}
impl<'de> Deserialize<'de> for SetMotorVelocityLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let v = RawTracingLog::<MotorVelocityLogFields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            velocity: v.fields.velocity,
        })
    }
}
impl<'de> Deserialize<'de> for GetMotorVelocityLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let v = RawTracingLog::<MotorVelocityLogFields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            velocity: v.fields.velocity,
        })
    }
}

// =============================================================================
// arci::MoveBase

// Currently, both SendVelocityLog and CurrentVelocityLog are the same.
// However, we may want to add result field to SendVelocityLog.
#[derive(Debug)]
#[non_exhaustive]
pub struct SendVelocityLog {
    pub timestamp: Timestamp,
    pub velocity: BaseVelocity,
}
#[derive(Debug)]
#[non_exhaustive]
pub struct CurrentVelocityLog {
    pub timestamp: Timestamp,
    pub velocity: BaseVelocity,
}

#[derive(Deserialize)]
struct VelocityLogFields {
    velocity_x: f64,
    velocity_y: f64,
    velocity_theta: f64,
}
impl<'de> Deserialize<'de> for SendVelocityLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let v = RawTracingLog::<VelocityLogFields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            velocity: BaseVelocity {
                x: v.fields.velocity_x,
                y: v.fields.velocity_y,
                theta: v.fields.velocity_theta,
            },
        })
    }
}
impl<'de> Deserialize<'de> for CurrentVelocityLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let v = RawTracingLog::<VelocityLogFields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            velocity: BaseVelocity {
                x: v.fields.velocity_x,
                y: v.fields.velocity_y,
                theta: v.fields.velocity_theta,
            },
        })
    }
}

// =============================================================================
// arci::Navigation

#[derive(Debug)]
#[non_exhaustive]
pub struct SendGoalPoseLog {
    pub timestamp: Timestamp,
    pub goal: arci::Isometry2<f64>,
    pub frame_id: String,
    pub timeout: Duration,
}
#[derive(Debug)]
#[non_exhaustive]
pub struct NavigationCancelLog {
    pub timestamp: Timestamp,
}

impl<'de> Deserialize<'de> for SendGoalPoseLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Fields {
            goal_rotation_re: f64,
            goal_rotation_im: f64,
            goal_translation_x: f64,
            goal_translation_y: f64,
            frame_id: String,
            timeout_secs: u64,
            timeout_nanos: u32,
        }
        let v = RawTracingLog::<Fields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            goal: arci::Isometry2::from_parts(
                nalgebra::Translation2::new(
                    v.fields.goal_translation_x,
                    v.fields.goal_translation_y,
                ),
                nalgebra::UnitComplex::from_complex(nalgebra::Complex {
                    re: v.fields.goal_rotation_re,
                    im: v.fields.goal_rotation_im,
                }),
            ),
            frame_id: v.fields.frame_id,
            timeout: Duration::new(v.fields.timeout_secs, v.fields.timeout_nanos),
        })
    }
}
impl<'de> Deserialize<'de> for NavigationCancelLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Fields {}
        let v = RawTracingLog::<Fields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
        })
    }
}

// =============================================================================
// arci::Speaker

#[derive(Debug)]
#[non_exhaustive]
pub struct SpeakLog {
    pub timestamp: Timestamp,
    pub message: String,
}

impl<'de> Deserialize<'de> for SpeakLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Fields {
            message: String,
        }
        let v = RawTracingLog::<Fields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            message: v.fields.message,
        })
    }
}

// =============================================================================
// arci::TransformResolver

#[derive(Debug)]
#[non_exhaustive]
pub struct ResolveTransformationLog {
    pub timestamp: Timestamp,
    pub from: String,
    pub to: String,
    pub time: SystemTime,
}

impl<'de> Deserialize<'de> for ResolveTransformationLog {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Fields {
            from: String,
            to: String,
            time_secs: u64,
            time_nanos: u32,
        }
        let v = RawTracingLog::<Fields>::deserialize(deserializer)?;
        Ok(Self {
            timestamp: v.timestamp,
            from: v.fields.from,
            to: v.fields.to,
            time: SystemTime::UNIX_EPOCH + Duration::new(v.fields.time_secs, v.fields.time_nanos),
        })
    }
}
