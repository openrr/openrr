#[tokio::main]
async fn main() {
    println!("listening on http://localhost:4000");

    warp::serve(warp::fs::dir("."))
        .run(([0, 0, 0, 0], 4000))
        .await;
}
