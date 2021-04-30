use schemars::schema_for;
use structopt::{clap::arg_enum, StructOpt};
use tracing::debug;

#[derive(Debug, StructOpt)]
#[structopt(name = env!("CARGO_BIN_NAME"))]
struct Args {
    #[structopt(subcommand)]
    subcommand: Subcommand,
}

#[derive(Debug, StructOpt)]
enum Subcommand {
    /// Generate JSON schema for the specified config file.
    Schema {
        /// Kind of config file.
        #[structopt(possible_values = &ConfigKind::variants(), case_insensitive = true)]
        kind: ConfigKind,
    },
}

arg_enum! {
    #[derive(Debug)]
    enum ConfigKind {
        RobotConfig,
        RobotTeleopConfig,
    }
}

fn main() {
    tracing_subscriber::fmt::init();
    let args = Args::from_args();
    debug!(?args);

    match args.subcommand {
        Subcommand::Schema { kind } => {
            let schema = match kind {
                ConfigKind::RobotConfig => schema_for!(openrr_apps::RobotConfig),
                ConfigKind::RobotTeleopConfig => schema_for!(openrr_apps::RobotTeleopConfig),
            };
            println!("{}", serde_json::to_string_pretty(&schema).unwrap());
        }
    }
}
