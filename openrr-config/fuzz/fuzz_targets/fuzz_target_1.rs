#![cfg_attr(easyfuzz_no_main, no_main)]

#[allow(dead_code)]
#[path = "../../src/lib.rs"]
mod openrr_config;

#[easyfuzz::fuzz]
fn fuzz(data: &str) {
    let _ = openrr_config::parse_scripts(data);
}
