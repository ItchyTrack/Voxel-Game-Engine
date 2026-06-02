use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::{unbounded, Receiver, Sender};

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;

#[derive(Debug, Clone, Copy)]
pub struct ChunkLoadRequest {
	pub grid: GridId,
	pub chunk: IVec3,
}

pub struct ChunkLoadResult {
	pub grid: GridId,
	pub chunk: IVec3,
	pub voxels: Option<Voxels>,
}

/// Outbound queue: requests pushed by the streaming systems, drained by the
/// user's loader (clone `receiver()` into an async/off-thread loader).
#[derive(Resource)]
pub struct ChunkRequestChannel {
	sender: Sender<ChunkLoadRequest>,
	receiver: Receiver<ChunkLoadRequest>,
}

impl Default for ChunkRequestChannel {
	fn default() -> Self {
		let (sender, receiver) = unbounded();
		Self { sender, receiver }
	}
}

impl ChunkRequestChannel {
	pub(crate) fn request(&self, request: ChunkLoadRequest) {
		let _ = self.sender.send(request);
	}

	pub fn receiver(&self) -> Receiver<ChunkLoadRequest> {
		self.receiver.clone()
	}

	pub fn try_recv(&self) -> Option<ChunkLoadRequest> {
		self.receiver.try_recv().ok()
	}
}

/// Inbound channel: finished loads pushed by the loader from any thread,
/// drained by the plugin.
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
