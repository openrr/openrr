// Check if the macro depends on another crate.

use arci_ros::msg;

arci_ros::define_action_client!(SimpleActionClient, msg::move_base_msgs, MoveBase);
