use std::sync::atomic::{AtomicU64, Ordering};

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::{Receiver, Sender, unbounded};
use serde::{Deserialize, Serialize};

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;

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

	pub fn receiver(&self) -> Receiver<ChunkSaveRequest> {
		self.receiver.clone()
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
pub struct LodKey {
	pub min: IVec3,
	pub size: IVec3,
	pub lod: u8,
}

#[derive(Debug, Clone, Copy)]
pub struct LodLoadRequest {
	pub grid: GridId,
	pub requester: Entity,
	pub key: LodKey,
	pub priority: f32,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct GeneratedChunkLoadRequest {
	pub request: ChunkLoadRequest,
	pub generation: u64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct GeneratedLodLoadRequest {
	pub request: LodLoadRequest,
	pub generation: u64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum SourceRequest {
	Presence(PresenceLoadRequest),
	Chunk(GeneratedChunkLoadRequest),
	Lod(GeneratedLodLoadRequest),
}

#[derive(Resource)]
pub(crate) struct SourceRequestChannel {
	sender: Sender<SourceRequest>,
	receiver: Receiver<SourceRequest>,
	presence_sent: AtomicU64,
	chunk_sent: AtomicU64,
	lod_sent: AtomicU64,
}

impl Default for SourceRequestChannel {
	fn default() -> Self {
		let (sender, receiver) = unbounded();
		Self {
			sender,
			receiver,
			presence_sent: AtomicU64::new(0),
			chunk_sent: AtomicU64::new(0),
			lod_sent: AtomicU64::new(0),
		}
	}
}

impl SourceRequestChannel {
	pub(crate) fn request_presence(&self, request: PresenceLoadRequest) {
		if self.sender.send(SourceRequest::Presence(request)).is_ok() {
			self.presence_sent.fetch_add(1, Ordering::Relaxed);
		}
	}

	pub(crate) fn request_chunk(&self, request: ChunkLoadRequest, generation: u64) {
		if self.sender.send(SourceRequest::Chunk(GeneratedChunkLoadRequest { request, generation })).is_ok() {
			self.chunk_sent.fetch_add(1, Ordering::Relaxed);
		}
	}

	pub(crate) fn request_lod(&self, request: LodLoadRequest, generation: u64) {
		if self.sender.send(SourceRequest::Lod(GeneratedLodLoadRequest { request, generation })).is_ok() {
			self.lod_sent.fetch_add(1, Ordering::Relaxed);
		}
	}

	pub(crate) fn receiver(&self) -> Receiver<SourceRequest> {
		self.receiver.clone()
	}

	pub(crate) fn chunk_sent_count(&self) -> u64 {
		self.chunk_sent.load(Ordering::Relaxed)
	}

	pub(crate) fn lod_sent_count(&self) -> u64 {
		self.lod_sent.load(Ordering::Relaxed)
	}
}

