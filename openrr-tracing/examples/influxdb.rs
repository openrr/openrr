// ```
// cd openrr-tracing
// docker run -p "8086:8086" --init --rm --name "influxdb_run" -it "influxdb:1.8"
// cargo run --example influxdb
// ```

use std::{ffi::OsStr, time::Duration};

use anyhow::Result;
use arci::*;
use fs_err as fs;
use openrr_tracing::de::TracingLog;

const LOG_DIR: &str = "./logs";

#[tokio::main]
async fn main() -> Result<()> {
    std::thread::spawn(|| {
        let file = tracing_appender::rolling::daily(LOG_DIR, "trace");
        tracing_subscriber::fmt()
            .json()
            .with_writer(file)
            .with_ansi(false)
            .with_max_level(tracing::Level::TRACE)
            .with_current_span(false)
            .init();

        main_thread()
    })
    .join()
    .unwrap()?;

    let mut send_velocity_logs = vec![];
    let mut current_pose_logs = vec![];
    for e in fs::read_dir(LOG_DIR)?.filter_map(Result::ok) {
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
                TracingLog::CurrentPose(log) => current_pose_logs.push(log),
                _ => todo!(),
            }
        }
    }

    let mut v = vec![];
    for log in send_velocity_logs.windows(2).rev() {
        use influxdb::InfluxDbWriteable;
        let d = log[1].timestamp.signed_duration_since(log[0].timestamp);
        v.push(
            influxdb::Timestamp::Nanoseconds(log[0].timestamp.timestamp_nanos_opt().unwrap() as _)
                .into_query("send_velocity")
                .add_field("duration", d.num_nanoseconds().unwrap())
                .add_field("velocity_x", log[0].velocity.x)
                .add_field("velocity_y", log[0].velocity.y)
                .add_field("velocity_theta", log[0].velocity.theta),
        );
    }

    let test_name = "test_send_velocity";
    let client = influxdb::Client::new("http://127.0.0.1:8086", test_name);
    let query = format!("CREATE DATABASE {}", test_name);
    client.query(influxdb::ReadQuery::new(query)).await.unwrap();
    for v in v.iter().rev() {
        let _res = client.query(v).await;
    }

    let read_query = influxdb::ReadQuery::new("SELECT * FROM send_velocity");
    let read_result = client.query(read_query).await;
    println!("{read_result:?}");

    Ok(())
}

fn main_thread() -> Result<()> {
    let base = openrr_tracing::Tracing::new(arci::DummyMoveBase::new());
    let mut loc = openrr_tracing::Tracing::new(arci::DummyLocalization::new());
    let mut vel = BaseVelocity {
        x: 1.0,
        y: 1.0,
        theta: 1.0,
    };
    base.send_velocity(&vel)?;
    loc.get_mut().current_pose.translation = [1.0, 1.0].into();
    loc.current_pose("")?;

    for _ in 0..10 {
        std::thread::sleep(Duration::from_secs(1));

        vel *= 1.1;
        base.send_velocity(&vel)?;
        loc.get_mut().current_pose.translation.x *= 1.1;
        loc.get_mut().current_pose.translation.y *= 1.2;
        loc.current_pose("")?;
    }

    Ok(())
}
