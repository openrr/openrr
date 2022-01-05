use arci::Speaker;
use arci_speak_cmd::LocalCommand;
use clap::Parser;

#[derive(Debug, Parser)]
struct Args {
    message: String,
}

#[tokio::main]
async fn main() -> Result<(), arci::Error> {
    tracing_subscriber::fmt::init();
    let args = Args::parse();

    let speaker = LocalCommand::default();
    let wait = speaker.speak(&args.message)?;
    wait.await
}
