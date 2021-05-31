use std::{env, time::SystemTime};

use anyhow::{anyhow, Error};
use arci::TransformResolver;
use arci_ros::Tf2BufferServerClient;

fn main() -> Result<(), Error> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(anyhow!("{} from_frame_id to_frame_id", args[0]));
    }
    let from = &args[1];
    let to = &args[2];

    arci_ros::init("arc_ros_example_resolve_transformation");
    let resolver = Tf2BufferServerClient::new("/tf2_buffer_server", 1, 10.0);
    let rate = arci_ros::rate(1.0);
    while arci_ros::is_ok() {
        let transformation = resolver.resolve_transformation(from, to, SystemTime::now());
        println!("{:?}", transformation);
        rate.sleep();
    }
    Ok(())
}
