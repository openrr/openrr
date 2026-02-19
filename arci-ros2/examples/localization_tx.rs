#[cfg(feature = "ros2")]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use arci_ros2::{
        r2r::{self, geometry_msgs::msg::*, std_msgs::msg::Header},
        Node,
    };

    let node = Node::new("localization_tx", "arci_ros2")?;
    let publisher = node
        .r2r()
        .create_publisher::<r2r::geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose",
            r2r::QosProfile::default(),
        )?;

    publisher.publish(&PoseWithCovarianceStamped {
        header: Header::default(),
        pose: PoseWithCovariance {
            pose: Pose {
                position: Point {
                    x: 2.0,
                    y: 3.0,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.5 * std::f64::consts::SQRT_2,
                    w: 0.5 * std::f64::consts::SQRT_2,
                },
            },
            covariance: vec![1f64; 36],
        },
    })?;

    Ok(())
}

#[cfg(not(feature = "ros2"))]
fn main() {
    println!("This example requires ros2 feature");
}
