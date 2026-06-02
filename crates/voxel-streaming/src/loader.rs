use bevy::ecs::message::Message;
use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::{unbounded, Receiver, Sender};

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;

#[derive(Message, Debug, Clone, Copy)]
pub struct ChunkLoadRequest {
	pub grid: GridId,
	pub chunk: IVec3,
}

pub struct ChunkLoadResult {
	pub grid: GridId,
	pub chunk: IVec3,
	pub voxels: Option<Voxels>,
}

#[derive(Resource)]
pub struct ChunkLoaderChannel {
	sender: Sender<ChunkLoadResult>,
	receiver: Receiver<ChunkLoadResult>,
}

impl Default for ChunkLoaderChannel {
	fn default() -> Self {
		let (sender, receiver) = unbounded();
		Self { sender, receiver }
	}
}

impl ChunkLoaderChannel {
	pub fn sender(&self) -> Sender<ChunkLoadResult> {
		self.sender.clone()
	}

	pub fn report(&self, result: ChunkLoadResult) {
		let _ = self.sender.send(result);
	}

	pub(crate) fn try_recv(&self) -> Option<ChunkLoadResult> {
		self.receiver.try_recv().ok()
	}
}
