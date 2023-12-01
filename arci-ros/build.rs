use std::path::PathBuf;

fn main() {
    let msg_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("ros_msgs");
    println!(
        "cargo:rustc-env=ROSRUST_MSG_PATH={}",
        msg_path.to_str().unwrap()
    )
}
