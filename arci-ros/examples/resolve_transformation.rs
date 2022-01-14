use std::{
    env,
    time::{Duration, SystemTime},
};

use anyhow::{anyhow, Error};
use arci::TransformResolver;
use arci_ros::RosTransformResolver;

const TF_RETRY_RATE: f64 = 10.0;
const TF_MAX_RETRY: usize = 10;
const TF_CACHE_DURATION_SECS: f64 = 10.0;
const TIME_OFFSET_SECS: f64 = 0.1;

fn main() -> Result<(), Error> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(anyhow!("{} from_frame_id to_frame_id", args[0]));
    }
    let from = &args[1];
    let to = &args[2];

    arci_ros::init("arc_ros_example_resolve_transformation");
    let resolver = RosTransformResolver::new(
        Duration::from_secs_f64(TF_CACHE_DURATION_SECS),
        TF_RETRY_RATE,
        TF_MAX_RETRY,
    );
    let rate = arci_ros::rate(1.0);
    while arci_ros::is_ok() {
        match resolver.resolve_transformation(
            from,
            to,
            SystemTime::now() - Duration::from_secs_f64(TIME_OFFSET_SECS),
        ) {
            Ok(transformation) => {
                let translation = transformation.translation;
                let rotation = transformation.rotation;
                let rpy = rotation.euler_angles();
                println!(
                    "Translation x: {:.3} y: {:.3} z: {:.3}",
                    translation.x, translation.y, translation.z,
                );
                println!("Rotation rpy: {:.3} {:.3} {:.3}", rpy.0, rpy.1, rpy.2,);
                println!(
                    "Rotation xyzw: {:.3} {:.3} {:.3} {:.3}",
                    rotation.i, rotation.j, rotation.k, rotation.w
                )
            }
            Err(e) => println!("{e:?}"),
        };
        rate.sleep();
    }
    Ok(())
}
