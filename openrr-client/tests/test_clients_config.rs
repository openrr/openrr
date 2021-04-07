use openrr_client::OpenrrClientsConfig;

#[test]
fn ser_default_config() {
    toml::to_string(&OpenrrClientsConfig::default()).unwrap();
}
