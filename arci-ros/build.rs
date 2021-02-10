use std::{env, path::PathBuf};

fn main() {
    let msg_path =
        PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("failed to get manifest root"))
            .join("ros_msgs");
    println!(
        "cargo:rustc-env=ROSRUST_MSG_PATH={}",
        msg_path.to_str().unwrap()
    )
}
