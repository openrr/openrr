#[cfg(target_os = "linux")]
use arci::{BaseVelocity, MoveBase};

#[cfg(target_os = "linux")]
mod util;

#[cfg(target_os = "linux")]
#[tokio::test]
async fn test_cmd_vel() {

	let _roscore = util::run_roscore_for(util::Language::None, util::Feature::Publisher);


	let topic_name = String::from("/cmd_vel");
	arci_ros::init("arci_ros_cmd_vel_test");

	let sub = rosrust::subscribe(&topic_name, 1, |v: arci_ros::msg::geometry_msgs::Twist| {
		println!("{:?}", v)
	})
	.unwrap();

	let topic_name = String::from("/cmd_vel");
	let c = arci_ros::RosCmdVelMoveBase::new(&topic_name);
	let mut vel = BaseVelocity::default();
	println!("test cmd_vel is running!");

	for count in 0..100 {
		vel.x = 0.001 * (count as f64);
		c.send_velocity(&vel).unwrap();
		std::thread::sleep(std::time::Duration::from_millis(100));
		println!("{}, {:?}", count, vel);
	}
}