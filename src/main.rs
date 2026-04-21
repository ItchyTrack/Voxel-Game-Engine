use voxel_game_engine::run;

use tracy_client::{Client};

#[tokio::main]
async fn main() {
	Client::start();

	let _ = run();
}
