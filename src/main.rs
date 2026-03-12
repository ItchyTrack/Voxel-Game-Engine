use voxel_game_engine::run;

use tracy_client::{Client};

fn main() {
	Client::start();

	let _ = run();
}
