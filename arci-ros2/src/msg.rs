#![allow(unreachable_pub, missing_docs, non_snake_case)]

use ros2_client::builtin_interfaces;

pub trait MessageType: Sized {
    fn message_type_name() -> ros2_client::MessageTypeName;
    fn action_type_name() -> ros2_client::ActionTypeName;
    fn service_type_name() -> ros2_client::ServiceTypeName;
}
macro_rules! message_type {
    ($($package_name:ident / $type_name:ident),* $(,)?) => {$(
        impl ros2_client::Message for crate::msg::$package_name::$type_name {}
        impl crate::msg::MessageType for crate::msg::$package_name::$type_name {
            fn message_type_name() -> ros2_client::MessageTypeName {
                ros2_client::MessageTypeName::new(stringify!($package_name), stringify!($type_name))
            }
            fn action_type_name() -> ros2_client::ActionTypeName {
                unimplemented!()
            }
            fn service_type_name() -> ros2_client::ServiceTypeName {
                unimplemented!()
            }
        }
    )*};
}
macro_rules! action_type {
    ($($package_name:ident / $action_name:ident),* $(,)?) => {$(
        pub type Action = ros2_client::action::Action<
            crate::msg::$package_name::$action_name::Goal,
            crate::msg::$package_name::$action_name::Result,
            crate::msg::$package_name::$action_name::Feedback,
        >;
        impl ros2_client::Message for crate::msg::$package_name::$action_name::Goal {}
        impl ros2_client::Message for crate::msg::$package_name::$action_name::Result {}
        impl ros2_client::Message for crate::msg::$package_name::$action_name::Feedback {}
        impl crate::msg::MessageType for crate::msg::$package_name::$action_name::Action {
            fn message_type_name() -> ros2_client::MessageTypeName {
                ros2_client::MessageTypeName::new(stringify!($package_name), stringify!($action_name))
            }
            fn action_type_name() -> ros2_client::ActionTypeName {
                ros2_client::ActionTypeName::new(stringify!($package_name), stringify!($action_name))
            }
            fn service_type_name() -> ros2_client::ServiceTypeName {
                unimplemented!()
            }
        }
    )*};
}
message_type!(
    std_msgs / Empty,
    geometry_msgs / Twist,
    geometry_msgs / PoseStamped,
    geometry_msgs / PoseWithCovarianceStamped,
    sensor_msgs / LaserScan,
    control_msgs / JointTrajectoryControllerState,
    tf2_msgs / TFMessage,
);

/// [std_msgs](https://github.com/ros2/common_interfaces/tree/HEAD/std_msgs)
pub mod std_msgs {
    use serde::{Deserialize, Serialize};

    use crate::msg::*;

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Empty {}

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Header {
        pub stamp: builtin_interfaces::Time,
        pub frame_id: String,
    }
    impl Default for Header {
        fn default() -> Self {
            Self {
                stamp: builtin_interfaces::Time::ZERO,
                frame_id: Default::default(),
            }
        }
    }
}

/// [std_srvs](https://github.com/ros2/common_interfaces/tree/HEAD/std_srvs)
pub mod std_srvs {
    use crate::msg::*;

    pub type Empty = ros2_client::AService<std_msgs::Empty, std_msgs::Empty>;

    impl MessageType for Empty {
        fn message_type_name() -> ros2_client::MessageTypeName {
            unimplemented!()
        }

        fn action_type_name() -> ros2_client::ActionTypeName {
            unimplemented!()
        }

        fn service_type_name() -> ros2_client::ServiceTypeName {
            ros2_client::ServiceTypeName::new("std_srvs", "Empty")
        }
    }
}

/// [geometry_msgs](https://github.com/ros2/common_interfaces/tree/HEAD/geometry_msgs)
pub mod geometry_msgs {
    use serde::{Deserialize, Serialize};

    use crate::msg::*;

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Twist {
        pub linear: Vector3,
        pub angular: Vector3,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct PoseWithCovarianceStamped {
        pub header: std_msgs::Header,
        pub pose: PoseWithCovariance,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct PoseWithCovariance {
        pub pose: Pose,
        pub covariance: Vec<f64>,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct PoseStamped {
        pub header: std_msgs::Header,
        pub pose: Pose,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Pose {
        pub position: Point,
        pub orientation: Quaternion,
    }
    impl From<&arci::Isometry2<f64>> for Pose {
        fn from(pose: &arci::Isometry2<f64>) -> Self {
            let q = arci::UnitQuaternion::from_axis_angle(
                &arci::Vector3::z_axis(),
                pose.rotation.angle(),
            );
            Pose {
                position: Point {
                    x: pose.translation.x,
                    y: pose.translation.y,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: q.coords.x,
                    y: q.coords.y,
                    z: q.coords.z,
                    w: q.coords.w,
                },
            }
        }
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Transform {
        pub translation: geometry_msgs::Vector3,
        pub rotation: geometry_msgs::Quaternion,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct TransformStamped {
        pub header: std_msgs::Header,
        pub child_frame_id: String,
        pub transform: Transform,
    }
}

/// [sensor_msgs](https://github.com/ros2/common_interfaces/tree/HEAD/sensor_msgs)
pub mod sensor_msgs {
    use serde::{Deserialize, Serialize};

    use crate::msg::*;

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct LaserScan {
        pub header: std_msgs::Header,
        pub angle_min: f32,
        pub angle_max: f32,
        pub angle_increment: f32,
        pub time_increment: f32,
        pub scan_time: f32,
        pub range_min: f32,
        pub range_max: f32,
        pub ranges: Vec<f32>,
        pub intensities: Vec<f32>,
    }
}

/// [trajectory_msgs](https://github.com/ros2/common_interfaces/tree/HEAD/trajectory_msgs)
pub mod trajectory_msgs {
    use serde::{Deserialize, Serialize};

    use crate::msg::*;

    #[derive(Clone, Debug, Default, Serialize, Deserialize)]
    #[serde(default)]
    pub struct JointTrajectory {
        pub header: std_msgs::Header,
        pub joint_names: Vec<String>,
        pub points: Vec<trajectory_msgs::JointTrajectoryPoint>,
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    #[serde(default)]
    pub struct JointTrajectoryPoint {
        pub positions: Vec<f64>,
        pub velocities: Vec<f64>,
        pub accelerations: Vec<f64>,
        pub effort: Vec<f64>,
        pub time_from_start: builtin_interfaces::Duration,
    }
    impl Default for JointTrajectoryPoint {
        fn default() -> Self {
            Self {
                positions: Default::default(),
                velocities: Default::default(),
                accelerations: Default::default(),
                effort: Default::default(),
                time_from_start: builtin_interfaces::Duration { sec: 0, nanosec: 0 },
            }
        }
    }

    #[derive(Clone, Debug, Default, Serialize, Deserialize)]
    #[serde(default)]
    pub struct MultiDOFJointTrajectory {
        pub header: std_msgs::Header,
        pub joint_names: Vec<String>,
        pub points: Vec<trajectory_msgs::MultiDOFJointTrajectoryPoint>,
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    #[serde(default)]
    pub struct MultiDOFJointTrajectoryPoint {
        pub transforms: Vec<geometry_msgs::Transform>,
        pub velocities: Vec<geometry_msgs::Twist>,
        pub accelerations: Vec<geometry_msgs::Twist>,
        pub time_from_start: builtin_interfaces::Duration,
    }
    impl Default for MultiDOFJointTrajectoryPoint {
        fn default() -> Self {
            Self {
                transforms: Default::default(),
                velocities: Default::default(),
                accelerations: Default::default(),
                time_from_start: builtin_interfaces::Duration { sec: 0, nanosec: 0 },
            }
        }
    }
}

/// [nav2_msgs](https://github.com/ros-planning/navigation2/tree/HEAD/nav2_msgs)
pub mod nav2_msgs {
    pub mod NavigateToPose {
        use serde::{Deserialize, Serialize};

        use crate::msg::*;

        action_type!(nav2_msgs / NavigateToPose);

        #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Goal {
            pub pose: geometry_msgs::PoseStamped,
            pub behavior_tree: String,
        }

        #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Result {
            pub result: std_msgs::Empty,
        }

        #[derive(Clone, Debug, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Feedback {
            pub current_pose: geometry_msgs::PoseStamped,
            pub navigation_time: builtin_interfaces::Duration,
            pub estimated_time_remaining: builtin_interfaces::Duration,
            pub number_of_recoveries: i16,
            pub distance_remaining: f32,
        }
        impl Default for Feedback {
            fn default() -> Self {
                Self {
                    current_pose: Default::default(),
                    navigation_time: builtin_interfaces::Duration { sec: 0, nanosec: 0 },
                    estimated_time_remaining: builtin_interfaces::Duration { sec: 0, nanosec: 0 },
                    number_of_recoveries: Default::default(),
                    distance_remaining: Default::default(),
                }
            }
        }
    }
}

/// [control_msgs](https://github.com/ros-controls/control_msgs/tree/HEAD/control_msgs)
pub mod control_msgs {
    pub mod FollowJointTrajectory {
        use serde::{Deserialize, Serialize};

        use crate::msg::*;

        action_type!(control_msgs / FollowJointTrajectory);

        #[derive(Clone, Debug, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Goal {
            pub trajectory: trajectory_msgs::JointTrajectory,
            pub multi_dof_trajectory: trajectory_msgs::MultiDOFJointTrajectory,
            pub path_tolerance: Vec<control_msgs::JointTolerance>,
            pub component_path_tolerance: Vec<control_msgs::JointComponentTolerance>,
            pub goal_tolerance: Vec<control_msgs::JointTolerance>,
            pub component_goal_tolerance: Vec<control_msgs::JointComponentTolerance>,
            pub goal_time_tolerance: builtin_interfaces::Duration,
        }
        impl Default for Goal {
            fn default() -> Self {
                Self {
                    trajectory: Default::default(),
                    multi_dof_trajectory: Default::default(),
                    path_tolerance: Default::default(),
                    component_path_tolerance: Default::default(),
                    goal_tolerance: Default::default(),
                    component_goal_tolerance: Default::default(),
                    goal_time_tolerance: builtin_interfaces::Duration { sec: 0, nanosec: 0 },
                }
            }
        }

        #[derive(Clone, Default, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Result {
            pub error_code: i32,
            pub error_string: String,
        }
        impl Result {
            pub const GOAL_TOLERANCE_VIOLATED: i32 = -5;
            pub const INVALID_GOAL: i32 = -1;
            pub const INVALID_JOINTS: i32 = -2;
            pub const OLD_HEADER_TIMESTAMP: i32 = -3;
            pub const PATH_TOLERANCE_VIOLATED: i32 = -4;
            pub const SUCCESSFUL: i32 = 0;
        }

        #[derive(Clone, Default, Debug, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Feedback {
            pub header: std_msgs::Header,
            pub joint_names: Vec<String>,
            pub desired: trajectory_msgs::JointTrajectoryPoint,
            pub actual: trajectory_msgs::JointTrajectoryPoint,
            pub error: trajectory_msgs::JointTrajectoryPoint,
            pub multi_dof_joint_names: Vec<String>,
            pub multi_dof_desired: trajectory_msgs::MultiDOFJointTrajectoryPoint,
            pub multi_dof_actual: trajectory_msgs::MultiDOFJointTrajectoryPoint,
            pub multi_dof_error: trajectory_msgs::MultiDOFJointTrajectoryPoint,
        }
    }

    use serde::{Deserialize, Serialize};

    use crate::msg::*;

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct JointTolerance {
        pub name: String,
        pub position: f64,
        pub velocity: f64,
        pub acceleration: f64,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct JointComponentTolerance {
        pub joint_name: std::string::String,
        pub component: u16,
        pub position: f64,
        pub velocity: f64,
        pub acceleration: f64,
    }

    #[derive(Clone, Debug, Default, Serialize, Deserialize)]
    #[serde(default)]
    pub struct JointTrajectoryControllerState {
        pub header: std_msgs::Header,
        pub joint_names: Vec<String>,
        pub reference: trajectory_msgs::JointTrajectoryPoint,
        pub feedback: trajectory_msgs::JointTrajectoryPoint,
        pub error: trajectory_msgs::JointTrajectoryPoint,
        pub output: trajectory_msgs::JointTrajectoryPoint,
        pub desired: trajectory_msgs::JointTrajectoryPoint,
        pub actual: trajectory_msgs::JointTrajectoryPoint,
        pub multi_dof_joint_names: Vec<String>,
        pub multi_dof_reference: trajectory_msgs::MultiDOFJointTrajectoryPoint,
        pub multi_dof_feedback: trajectory_msgs::MultiDOFJointTrajectoryPoint,
        pub multi_dof_error: trajectory_msgs::MultiDOFJointTrajectoryPoint,
        pub multi_dof_output: trajectory_msgs::MultiDOFJointTrajectoryPoint,
        pub multi_dof_desired: trajectory_msgs::MultiDOFJointTrajectoryPoint,
        pub multi_dof_actual: trajectory_msgs::MultiDOFJointTrajectoryPoint,
    }
}

/// [tf2_msgs](https://github.com/ros2/geometry2/tree/HEAD/tf2_msgs)
pub mod tf2_msgs {
    use serde::{Deserialize, Serialize};

    use crate::msg::*;

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct TFMessage {
        pub transforms: Vec<geometry_msgs::TransformStamped>,
    }
}
