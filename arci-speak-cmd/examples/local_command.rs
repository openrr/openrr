use arci::Speaker;
use arci_speak_cmd::LocalCommand;
use structopt::StructOpt;

#[derive(Debug, StructOpt)]
struct Args {
    message: String,
}

#[tokio::main]
async fn main() -> Result<(), arci::Error> {
    tracing_subscriber::fmt::init();
    let args = Args::from_args();

    let speaker = LocalCommand::default();
    let wait = speaker.speak(&args.message)?;
    wait.await
}
