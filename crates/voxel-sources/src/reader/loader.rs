use std::sync::Arc;

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::{Receiver, Sender, unbounded};
use serde::{Deserialize, Serialize};

use tile_data::{TileGenerator, TileKey};
use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_tasks::CancellationToken;

use super::request_handle::VoxelSourceRequesterId;

#[derive(Debug)]
pub struct ChunkSaveRequest {
	pub grid: GridId,
	pub chunk: IVec3,
	pub voxels: Voxels,
}

#[derive(Resource)]
pub struct ChunkSaveChannel {
	sender: Sender<ChunkSaveRequest>,
	receiver: Receiver<ChunkSaveRequest>,
}

impl Default for ChunkSaveChannel {
	fn default() -> Self {
		let (sender, receiver) = unbounded();
		Self { sender, receiver }
	}
}

impl ChunkSaveChannel {
	pub fn save(&self, request: ChunkSaveRequest) {
		let _ = self.sender.send(request);
	}

	pub fn try_recv(&self) -> Option<ChunkSaveRequest> {
		self.receiver.try_recv().ok()
	}
}

#[derive(Debug, Clone, Copy)]
pub struct PresenceLoadRequest {
	pub grid: GridId,
}

#[derive(Debug, Clone, Copy)]
pub struct ChunkLoadRequest {
	pub grid: GridId,
	pub chunk: IVec3,
}

#[derive(Debug, Clone, Copy)]
pub struct TileLoadRequest {
	pub grid: GridId,
	pub requester: Entity,
	pub key: TileKey,
	pub tag: u64,
	pub priority: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TileVoxelKey {
	pub min: IVec3,
	pub size: IVec3,
	pub lod: u8,
}

#[derive(Clone)]
pub struct TileVoxelLoadRequest {
	pub grid: GridId,
	pub requester: Entity,
	pub key: TileVoxelKey,
	pub voxel_type: VoxelTypeId,
	/// Opaque caller value preserved in the completion message.
	pub tag: u64,
	pub priority: f32,
	pub(crate) tile: Option<TileCompletion>,
}

impl TileVoxelLoadRequest {
	pub fn raw(grid: GridId, requester: Entity, key: TileVoxelKey, voxel_type: VoxelTypeId, tag: u64, priority: f32) -> Self {
		Self { grid, requester, key, voxel_type, tag, priority, tile: None }
	}
}

#[derive(Clone)]
pub(crate) struct TileCompletion {
	pub key: TileKey,
	pub generator: Arc<dyn TileGenerator>,
}

pub(crate) struct PendingTileVoxelRequest {
	pub requester: VoxelSourceRequesterId,
	pub request: TileVoxelLoadRequest,
	pub cancellation: CancellationToken,
}

pub(crate) enum ReaderRequest {
	Presence { requester: VoxelSourceRequesterId, request: PresenceLoadRequest },
	Chunk { requester: VoxelSourceRequesterId, request: ChunkLoadRequest, cancellation: CancellationToken },
	TileVoxels { requester: VoxelSourceRequesterId, request: TileVoxelLoadRequest, cancellation: CancellationToken },
	CancelTileVoxels { requester: VoxelSourceRequesterId, grid: GridId, key: TileVoxelKey, voxel_type: VoxelTypeId },
	ReleaseRequester { requester: VoxelSourceRequesterId },
}

#[derive(Debug)]
pub struct TileVoxelCancellation {
	requester: VoxelSourceRequesterId,
	grid: GridId,
	key: TileVoxelKey,
	voxel_type: VoxelTypeId,
	cancellation: CancellationToken,
	sender: Sender<ReaderRequest>,
}

impl TileVoxelCancellation {
	pub(crate) fn new(
		requester: VoxelSourceRequesterId,
		grid: GridId,
		key: TileVoxelKey,
		voxel_type: VoxelTypeId,
		cancellation: CancellationToken,
		sender: Sender<ReaderRequest>,
	) -> Self {
		Self { requester, grid, key, voxel_type, cancellation, sender }
	}

	pub fn cancel(self) {
		self.cancellation.cancel();
		let _ = self.sender.send(ReaderRequest::CancelTileVoxels {
			requester: self.requester,
			grid: self.grid,
			key: self.key,
			voxel_type: self.voxel_type,
		});
	}
}
