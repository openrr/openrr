/*
This benchmark mainly measures the cost of transformations performed internally by the proxy types.
It is expected that the difference between `no_proxy_*` and `proxy_*` roughly indicates its cost.

$ cargo bench -p openrr-plugin | tee bench.txt

Benchmarking no_proxy_joint_names
Benchmarking no_proxy_joint_names: Warming up for 3.0000 s
Benchmarking no_proxy_joint_names: Collecting 100 samples in estimated 5.0119 s (475k iterations)
Benchmarking no_proxy_joint_names: Analyzing
no_proxy_joint_names    time:   [10.514 us 10.679 us 10.868 us]
Found 4 outliers among 100 measurements (4.00%)
  1 (1.00%) high mild
  3 (3.00%) high severe

Benchmarking no_proxy_current_joint_positions
Benchmarking no_proxy_current_joint_positions: Warming up for 3.0000 s
Benchmarking no_proxy_current_joint_positions: Collecting 100 samples in estimated 5.0005 s (44M iterations)
Benchmarking no_proxy_current_joint_positions: Analyzing
no_proxy_current_joint_positions
                        time:   [109.37 ns 110.84 ns 112.69 ns]
Found 8 outliers among 100 measurements (8.00%)
  6 (6.00%) high mild
  2 (2.00%) high severe

Benchmarking no_proxy_send_joint_positions
Benchmarking no_proxy_send_joint_positions: Warming up for 3.0000 s
Benchmarking no_proxy_send_joint_positions: Collecting 100 samples in estimated 5.0008 s (30M iterations)
Benchmarking no_proxy_send_joint_positions: Analyzing
no_proxy_send_joint_positions
                        time:   [165.42 ns 167.65 ns 170.19 ns]
Found 9 outliers among 100 measurements (9.00%)
  1 (1.00%) high mild
  8 (8.00%) high severe

Benchmarking proxy_same_crate_joint_names
Benchmarking proxy_same_crate_joint_names: Warming up for 3.0000 s
Benchmarking proxy_same_crate_joint_names: Collecting 100 samples in estimated 5.0410 s (404k iterations)
Benchmarking proxy_same_crate_joint_names: Analyzing
proxy_same_crate_joint_names
                        time:   [12.452 us 12.658 us 12.906 us]
Found 10 outliers among 100 measurements (10.00%)
  4 (4.00%) high mild
  6 (6.00%) high severe

Benchmarking proxy_same_crate_current_joint_positions
Benchmarking proxy_same_crate_current_joint_positions: Warming up for 3.0000 s
Benchmarking proxy_same_crate_current_joint_positions: Collecting 100 samples in estimated 5.0010 s (1.1M iterations)
Benchmarking proxy_same_crate_current_joint_positions: Analyzing
proxy_same_crate_current_joint_positions
                        time:   [4.4270 us 4.4548 us 4.4869 us]
Found 6 outliers among 100 measurements (6.00%)
  4 (4.00%) high mild
  2 (2.00%) high severe

Benchmarking proxy_same_crate_send_joint_positions
Benchmarking proxy_same_crate_send_joint_positions: Warming up for 3.0000 s
Benchmarking proxy_same_crate_send_joint_positions: Collecting 100 samples in estimated 5.0027 s (3.8M iterations)
Benchmarking proxy_same_crate_send_joint_positions: Analyzing
proxy_same_crate_send_joint_positions
                        time:   [1.2994 us 1.3079 us 1.3172 us]
Found 9 outliers among 100 measurements (9.00%)
  3 (3.00%) high mild
  6 (6.00%) high severe

   Compiling test_plugin v0.0.0 (/Users/taiki/projects/sources/smilerobotics/openrr/target/test_plugin)
    Finished release [optimized] target(s) in 1.55s
Benchmarking proxy_diff_crate_joint_names
Benchmarking proxy_diff_crate_joint_names: Warming up for 3.0000 s
Benchmarking proxy_diff_crate_joint_names: Collecting 100 samples in estimated 5.0832 s (288k iterations)
Benchmarking proxy_diff_crate_joint_names: Analyzing
proxy_diff_crate_joint_names
                        time:   [17.544 us 17.732 us 17.999 us]
Found 5 outliers among 100 measurements (5.00%)
  3 (3.00%) high mild
  2 (2.00%) high severe

   Compiling test_plugin v0.0.0 (/Users/taiki/projects/sources/smilerobotics/openrr/target/test_plugin)
    Finished release [optimized] target(s) in 1.50s
Benchmarking proxy_diff_crate_current_joint_positions
Benchmarking proxy_diff_crate_current_joint_positions: Warming up for 3.0000 s
Benchmarking proxy_diff_crate_current_joint_positions: Collecting 100 samples in estimated 5.0029 s (1.1M iterations)
Benchmarking proxy_diff_crate_current_joint_positions: Analyzing
proxy_diff_crate_current_joint_positions
                        time:   [4.4913 us 4.5279 us 4.5720 us]
Found 6 outliers among 100 measurements (6.00%)
  2 (2.00%) high mild
  4 (4.00%) high severe

   Compiling test_plugin v0.0.0 (/Users/taiki/projects/sources/smilerobotics/openrr/target/test_plugin)
    Finished release [optimized] target(s) in 1.45s
Benchmarking proxy_diff_crate_send_joint_positions
Benchmarking proxy_diff_crate_send_joint_positions: Warming up for 3.0000 s
Benchmarking proxy_diff_crate_send_joint_positions: Collecting 100 samples in estimated 5.0039 s (3.9M iterations)
Benchmarking proxy_diff_crate_send_joint_positions: Analyzing
proxy_diff_crate_send_joint_positions
                        time:   [1.2310 us 1.2537 us 1.2792 us]
Found 15 outliers among 100 measurements (15.00%)
  4 (4.00%) low mild
  3 (3.00%) high mild
  8 (8.00%) high severe

*/

use std::{env, path::PathBuf, process::Command, time::Duration};

use anyhow::Result;
use arci::{DummyJointTrajectoryClient, JointTrajectoryClient};
use criterion::{criterion_group, criterion_main, Criterion};
use openrr_plugin::{JointTrajectoryClientProxy, PluginProxy};

fn no_proxy_joint_names(c: &mut Criterion) {
    let joint_names: Vec<_> = (0..100).map(|n| n.to_string()).collect();
    let client = DummyJointTrajectoryClient::new(joint_names);
    c.bench_function("no_proxy_joint_names", |b| b.iter(|| client.joint_names()));
}

fn no_proxy_current_joint_positions(c: &mut Criterion) {
    let joint_names: Vec<_> = (0..100).map(|n| n.to_string()).collect();
    let client = DummyJointTrajectoryClient::new(joint_names);
    c.bench_function("no_proxy_current_joint_positions", |b| {
        b.iter(|| client.current_joint_positions().unwrap())
    });
}

fn no_proxy_send_joint_positions(c: &mut Criterion) {
    let joint_names: Vec<_> = (0..100).map(|n| n.to_string()).collect();
    let positions: Vec<_> = (0..joint_names.len()).map(|n| n as f64).collect();
    let client = DummyJointTrajectoryClient::new(joint_names);
    c.bench_function("no_proxy_send_joint_positions", |b| {
        b.iter(|| {
            client
                .send_joint_positions(positions.clone(), Duration::default())
                .unwrap()
        })
    });
}

fn proxy_same_crate_joint_names(c: &mut Criterion) {
    let joint_names: Vec<_> = (0..100).map(|n| n.to_string()).collect();
    let client = DummyJointTrajectoryClient::new(joint_names);
    let client = JointTrajectoryClientProxy::new(client);
    c.bench_function("proxy_same_crate_joint_names", |b| {
        b.iter(|| client.joint_names())
    });
}

fn proxy_same_crate_current_joint_positions(c: &mut Criterion) {
    let joint_names: Vec<_> = (0..100).map(|n| n.to_string()).collect();
    let client = DummyJointTrajectoryClient::new(joint_names);
    let client = JointTrajectoryClientProxy::new(client);
    c.bench_function("proxy_same_crate_current_joint_positions", |b| {
        b.iter(|| client.current_joint_positions().unwrap())
    });
}

fn proxy_same_crate_send_joint_positions(c: &mut Criterion) {
    let joint_names: Vec<_> = (0..100).map(|n| n.to_string()).collect();
    let positions: Vec<_> = (0..joint_names.len()).map(|n| n as f64).collect();
    let client = DummyJointTrajectoryClient::new(joint_names);
    let client = JointTrajectoryClientProxy::new(client);
    c.bench_function("proxy_same_crate_send_joint_positions", |b| {
        b.iter(|| {
            client
                .send_joint_positions(positions.clone(), Duration::default())
                .unwrap()
        })
    });
}

fn proxy_diff_crate_joint_names(c: &mut Criterion) {
    let plugin_path = test_plugin().unwrap();
    let plugin = PluginProxy::from_path(&plugin_path).unwrap();

    let joint_names: Vec<_> = (0..100).map(|n| n.to_string()).collect();
    let client = plugin
        .new_joint_trajectory_client(format!(r#"{{ "joint_names": {:?} }}"#, joint_names))
        .unwrap()
        .unwrap();

    c.bench_function("proxy_diff_crate_joint_names", |b| {
        b.iter(|| client.joint_names())
    });
}

fn proxy_diff_crate_current_joint_positions(c: &mut Criterion) {
    let plugin_path = test_plugin().unwrap();
    let plugin = PluginProxy::from_path(&plugin_path).unwrap();

    let joint_names: Vec<_> = (0..100).map(|n| n.to_string()).collect();
    let client = plugin
        .new_joint_trajectory_client(format!(r#"{{ "joint_names": {:?} }}"#, joint_names))
        .unwrap()
        .unwrap();

    c.bench_function("proxy_diff_crate_current_joint_positions", |b| {
        b.iter(|| client.current_joint_positions().unwrap())
    });
}

fn proxy_diff_crate_send_joint_positions(c: &mut Criterion) {
    let plugin_path = test_plugin().unwrap();
    let plugin = PluginProxy::from_path(&plugin_path).unwrap();

    let joint_names: Vec<_> = (0..100).map(|n| n.to_string()).collect();
    let positions: Vec<_> = (0..joint_names.len()).map(|n| n as f64).collect();
    let client = plugin
        .new_joint_trajectory_client(format!(r#"{{ "joint_names": {:?} }}"#, joint_names))
        .unwrap()
        .unwrap();

    c.bench_function("proxy_diff_crate_send_joint_positions", |b| {
        b.iter(|| {
            client
                .send_joint_positions(positions.clone(), Duration::default())
                .unwrap()
        })
    });
}

fn root_dir() -> PathBuf {
    let mut dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    dir.pop(); // openrr-plugin
    dir
}

fn test_plugin() -> Result<PathBuf> {
    let root_dir = root_dir();
    let cargo_toml = format!(
        r#"
[package]
name = "test_plugin"
version = "0.0.0"
edition = "2018"
publish = false

[workspace]
resolver = "2"

[lib]
crate-type = ["cdylib"]

[dependencies]
abi_stable = "0.9"
arci = {{ path = "{0}/arci" }}
openrr-plugin = {{ path = "{0}/openrr-plugin" }}
serde = {{ version = "1", features = ["derive"] }}
serde_json = "1"

[patch.crates-io]
arci = {{ path = "{0}/arci" }}
openrr-plugin = {{ path = "{0}/openrr-plugin" }}
    "#,
        root_dir.display()
    );
    let lib_rs = r#"
use arci::DummyJointTrajectoryClient;
use serde::Deserialize;

openrr_plugin::export_plugin!(TestPlugin);

pub struct TestPlugin;

impl openrr_plugin::Plugin for TestPlugin {
    fn name(&self) -> String {
        "TestPlugin".into()
    }

    fn new_joint_trajectory_client(
        &self,
        args: String,
    ) -> Result<Option<Box<dyn arci::JointTrajectoryClient>>, arci::Error> {
        let config: TestClientConfig =
            serde_json::from_str(&args).map_err(|e| arci::Error::Other(e.into()))?;
        Ok(Some(Box::new(DummyJointTrajectoryClient::new(config.joint_names))))
    }
}

#[derive(Deserialize)]
struct TestClientConfig {
    joint_names: Vec<String>,
}
    "#;

    let tmpdir_path = root_dir.join("target/test_plugin");
    for (path, contents) in &[
        ("src", None),
        ("src/lib.rs", Some(lib_rs)),
        ("Cargo.toml", Some(&*cargo_toml)),
    ] {
        let tmppath = &tmpdir_path.join(path);
        if let Some(contents) = contents {
            fs::write(tmppath, contents)?;
        } else if !tmppath.exists() {
            fs::create_dir_all(tmppath)?;
        }
    }

    let status = Command::new("cargo")
        .args(&["build", "--release", "--manifest-path"])
        .arg(&tmpdir_path.join("Cargo.toml"))
        .status()?;
    assert!(status.success());

    let plugin_path = tmpdir_path
        .join("target/release")
        .join("libtest_plugin.dylib");
    Ok(plugin_path)
}

criterion_group!(
    benches,
    no_proxy_joint_names,
    no_proxy_current_joint_positions,
    no_proxy_send_joint_positions,
    proxy_same_crate_joint_names,
    proxy_same_crate_current_joint_positions,
    proxy_same_crate_send_joint_positions,
    proxy_diff_crate_joint_names,
    proxy_diff_crate_current_joint_positions,
    proxy_diff_crate_send_joint_positions
);
criterion_main!(benches);
