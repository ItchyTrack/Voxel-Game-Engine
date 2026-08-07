use std::sync::{
	Arc,
	atomic::{AtomicU64, Ordering},
};

use bevy::prelude::*;
use crossbeam_channel::Sender;
use tile_data::TileGenerator;
use voxel_tasks::CancellationToken;

use super::VoxelReader;
use super::loader::{
	ChunkLoadRequest, PresenceLoadRequest, ReaderRequest, TileCompletion, TileLoadRequest,
	TileVoxelCancellation, TileVoxelKey, TileVoxelLoadRequest,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct VoxelSourceRequesterId(pub(crate) u64);

/// Information needed to allocate distinct voxel-source requester handles.
#[derive(Resource)]
pub struct VoxelSourcesRequestHandleGetter {
	sender: Sender<ReaderRequest>,
	next_id: AtomicU64,
}

impl FromWorld for VoxelSourcesRequestHandleGetter {
	fn from_world(world: &mut World) -> Self {
		Self {
			sender: world.resource::<VoxelReader>().request_sender(),
			next_id: AtomicU64::new(1),
		}
	}
}

impl VoxelSourcesRequestHandleGetter {
	pub fn get(&self) -> VoxelSourcesRequestHandle {
		VoxelSourcesRequestHandle {
			id: VoxelSourceRequesterId(self.next_id.fetch_add(1, Ordering::Relaxed)),
			sender: self.sender.clone(),
		}
	}
}

/// RAII requester identity and convenient access to voxel-source request functions.
pub struct VoxelSourcesRequestHandle {
	id: VoxelSourceRequesterId,
	sender: Sender<ReaderRequest>,
}

impl VoxelSourcesRequestHandle {
	pub fn request_presence(&self, request: PresenceLoadRequest) {
		let _ = self.sender.send(ReaderRequest::Presence { requester: self.id, request });
	}

	pub fn request_chunk(&self, request: ChunkLoadRequest) -> CancellationToken {
		let cancellation = CancellationToken::new();
		let _ = self.sender.send(ReaderRequest::Chunk {
			requester: self.id,
			request,
			cancellation: cancellation.clone(),
		});
		cancellation
	}

	pub fn request_tile_voxels(&self, request: TileVoxelLoadRequest) -> TileVoxelCancellation {
		let cancellation = CancellationToken::new();
		let result = TileVoxelCancellation::new(
			self.id,
			request.grid,
			request.key,
			request.voxel_type,
			cancellation.clone(),
			self.sender.clone(),
		);
		let _ = self.sender.send(ReaderRequest::TileVoxels {
			requester: self.id,
			request,
			cancellation,
		});
		result
	}

	pub fn request_tile(&self, request: TileLoadRequest, generator: Arc<dyn TileGenerator>) -> TileVoxelCancellation {
		let voxel_lod = request.key.lod.saturating_sub(generator.lod_levels());
		self.request_tile_voxels(TileVoxelLoadRequest {
			grid: request.grid,
			requester: request.requester,
			key: TileVoxelKey { min: request.key.min, size: request.key.size, lod: voxel_lod },
			voxel_type: generator.voxel_type(),
			tag: request.tag,
			priority: request.priority,
			tile: Some(TileCompletion { key: request.key, generator }),
		})
	}
}

impl Drop for VoxelSourcesRequestHandle {
	fn drop(&mut self) {
		let _ = self.sender.send(ReaderRequest::ReleaseRequester { requester: self.id });
	}
}
