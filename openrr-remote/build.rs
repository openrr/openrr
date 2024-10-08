use std::env;

fn main() {
    println!("cargo:rerun-if-changed=proto");
    println!("cargo:rerun-if-env-changed=OPENRR_REMOTE_LOCAL_OUT_DIR");
    println!("cargo:rustc-check-cfg=cfg(local_out_dir)");

    let mut config = tonic_build::configure();
    if env::var_os("OPENRR_REMOTE_LOCAL_OUT_DIR").is_some() {
        config = config.out_dir("src/generated");
        println!("cargo:rustc-cfg=local_out_dir");
    }
    config
        .compile_protos(&["proto/arci.proto"], &["proto"])
        .unwrap();
}
