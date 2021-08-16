fn main() {
    tonic_build::configure()
        .out_dir("src/generated")
        .compile(&["proto/arci.proto"], &["proto"])
        .unwrap();

    println!("cargo:rerun-if-changed=proto");
}
