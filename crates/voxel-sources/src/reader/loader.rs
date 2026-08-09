use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::{Receiver, Sender, unbounded};
use futures::channel::mpsc::UnboundedSender;
use serde::{Deserialize, Serialize};

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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct VoxelAreaKey {
	pub min: IVec3,
	pub size: IVec3,
	pub lod: u8,
}

#[derive(Clone)]
pub struct VoxelAreaLoadRequest {
	pub grid: GridId,
	pub key: VoxelAreaKey,
	pub voxel_type: VoxelTypeId,
	pub priority: f32,
}

#[derive(Clone)]
pub struct VoxelAreaMessageRequest {
	pub grid: GridId,
	pub requester: Entity,
	pub key: VoxelAreaKey,
	pub voxel_type: VoxelTypeId,
	/// Opaque caller value preserved in message-based completions.
	pub tag: u64,
	pub priority: f32,
}

impl VoxelAreaMessageRequest {
	pub fn raw(grid: GridId, requester: Entity, key: VoxelAreaKey, voxel_type: VoxelTypeId, tag: u64, priority: f32) -> Self {
		Self { grid, requester, key, voxel_type, tag, priority }
	}

	pub(crate) fn area_request(&self) -> VoxelAreaLoadRequest {
		VoxelAreaLoadRequest {
			grid: self.grid,
			key: self.key,
			voxel_type: self.voxel_type,
			priority: self.priority,
		}
	}
}

#[derive(Debug)]
pub struct VoxelAreaLoadResult {
	pub grid: GridId,
	pub key: VoxelAreaKey,
	pub voxel_type: VoxelTypeId,
	pub generation: u64,
	pub voxels: Option<Voxels>,
}

#[derive(Debug)]
pub enum VoxelAreaLoadEvent {
	Loaded(VoxelAreaLoadResult),
	Cancelled,
}

pub(crate) enum VoxelCompletionTarget {
	Message { requester: Entity, tag: u64 },
	Channel(UnboundedSender<VoxelAreaLoadEvent>),
}

pub(crate) struct PendingVoxelAreaRequest {
	pub requester: VoxelSourceRequesterId,
	pub request: VoxelAreaLoadRequest,
	pub cancellation: CancellationToken,
	pub target: VoxelCompletionTarget,
}

pub(crate) enum ReaderRequest {
	Presence { requester: VoxelSourceRequesterId, request: PresenceLoadRequest },
	Chunk { requester: VoxelSourceRequesterId, request: ChunkLoadRequest, cancellation: CancellationToken },
	VoxelArea {
		requester: VoxelSourceRequesterId,
		request: VoxelAreaLoadRequest,
		cancellation: CancellationToken,
		target: VoxelCompletionTarget,
	},
	CancelVoxelArea { requester: VoxelSourceRequesterId, grid: GridId, key: VoxelAreaKey, voxel_type: VoxelTypeId },
	ReleaseRequester { requester: VoxelSourceRequesterId },
}

#[derive(Debug)]
pub struct VoxelAreaCancellation {
	requester: VoxelSourceRequesterId,
	grid: GridId,
	key: VoxelAreaKey,
	voxel_type: VoxelTypeId,
	cancellation: CancellationToken,
	sender: Sender<ReaderRequest>,
}

impl VoxelAreaCancellation {
	pub(crate) fn new(
		requester: VoxelSourceRequesterId,
		grid: GridId,
		key: VoxelAreaKey,
		voxel_type: VoxelTypeId,
		cancellation: CancellationToken,
		sender: Sender<ReaderRequest>,
	) -> Self {
		Self { requester, grid, key, voxel_type, cancellation, sender }
	}

	pub fn cancel(self) {
		self.cancellation.cancel();
		let _ = self.sender.send(ReaderRequest::CancelVoxelArea {
			requester: self.requester,
			grid: self.grid,
			key: self.key,
			voxel_type: self.voxel_type,
		});
	}
}
