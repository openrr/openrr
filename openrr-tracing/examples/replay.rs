// ```
// # set OPENRR_TRACING_DIR
// export OPENRR_TRACING_DIR='./logs'
// # run urdf-viz
// urdf-viz ./openrr-planner/sample.urdf &
// # play with velocity sender
// cargo run -p openrr-apps --bin openrr_apps_velocity_sender -- --config-path ./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml --config openrr_tracing_config.move_base=true &
// # ...
// # close velocity sender
// # press "l" on urdf-viz to reset the location of the robot
// # run replay
// cargo run -p openrr-tracing --example replay -- --config-path ./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml
// ```

use std::{ffi::OsStr, path::PathBuf};

use anyhow::Result;
use arci::MoveBase;
use clap::Parser;
use fs_err as fs;
use openrr_client::BoxRobotClient;
use openrr_tracing::de::TracingLog;

#[derive(Parser, Debug)]
#[clap(name = env!("CARGO_BIN_NAME"))]
struct Opt {
    /// Path to the setting file.
    #[clap(short, long, value_parser)]
    config_path: Option<PathBuf>,
    /// Set options from command line. These settings take priority over the
    /// setting file specified by --config-path.
    #[clap(long)]
    config: Option<String>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let opt = Opt::parse();

    let log_dir = std::env::var_os("OPENRR_TRACING_DIR").expect("OPENRR_TRACING_DIR not set");
    let mut send_velocity_logs = vec![];
    for e in fs::read_dir(log_dir)?.filter_map(Result::ok) {
        let p = e.path();
        let stem = p.file_name().and_then(OsStr::to_str).unwrap();
        let _date = match stem.strip_prefix("trace.") {
            Some(date) => date,
            None => continue,
        };
        for log in openrr_tracing::de::from_str(&fs::read_to_string(p)?)? {
            println!("{log:?}");
            match log {
                TracingLog::SendVelocity(log) => send_velocity_logs.push(log),
                _ => todo!(),
            }
        }
    }

    let config_path = openrr_apps::utils::get_apps_robot_config(opt.config_path);
    let config =
        openrr_apps::utils::resolve_robot_config(config_path.as_deref(), opt.config.as_deref())?;

    openrr_apps::utils::init(env!("CARGO_BIN_NAME"), &config);
    let client: BoxRobotClient = config.create_robot_client()?;

    let mut v = vec![];
    for log in send_velocity_logs.windows(2).rev() {
        let d = log[1]
            .timestamp
            .signed_duration_since(log[0].timestamp)
            .to_std()?;
        v.push((log[0].timestamp, d, log[0].velocity));
    }
    v.reverse();

    for (_, duration, vel) in v {
        client.send_velocity(&vel)?;
        // TODO: should consider duration spend by the above send_velocity.
        tokio::time::sleep(duration).await;
    }

    Ok(())
}
