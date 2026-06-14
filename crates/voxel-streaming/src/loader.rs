use std::sync::atomic::{AtomicU64, Ordering};

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::{unbounded, Receiver, Sender};

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
	pub(crate) fn save(&self, request: ChunkSaveRequest) {
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
pub struct ChunkLoadRequest {
	pub grid: GridId,
	pub chunk: IVec3,
}

#[derive(Resource)]
pub struct ChunkRequestChannel {
	sender: Sender<ChunkLoadRequest>,
	receiver: Receiver<ChunkLoadRequest>,
	sent: AtomicU64,
}

impl Default for ChunkRequestChannel {
	fn default() -> Self {
		let (sender, receiver) = unbounded();
		Self { sender, receiver, sent: AtomicU64::new(0) }
	}
}

impl ChunkRequestChannel {
	pub(crate) fn request(&self, request: ChunkLoadRequest) {
		if self.sender.send(request).is_ok() {
			self.sent.fetch_add(1, Ordering::Relaxed);
		}
	}

	pub fn receiver(&self) -> Receiver<ChunkLoadRequest> {
		self.receiver.clone()
	}

	pub fn try_recv(&self) -> Option<ChunkLoadRequest> {
		self.receiver.try_recv().ok()
	}

	pub fn sent_count(&self) -> u64 {
		self.sent.load(Ordering::Relaxed)
	}
}

#[derive(Debug)]
pub struct ChunkLoadResult {
	pub grid: GridId,
	pub chunk: IVec3,
	pub voxels: Option<Voxels>,
}

#[derive(Resource)]
pub struct ChunkLoaderChannel {
	sender: Sender<ChunkLoadResult>,
	receiver: Receiver<ChunkLoadResult>,
	received: AtomicU64,
}

impl Default for ChunkLoaderChannel {
	fn default() -> Self {
		let (sender, receiver) = unbounded();
		Self { sender, receiver, received: AtomicU64::new(0) }
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
		let result = self.receiver.try_recv().ok();
		if result.is_some() {
			self.received.fetch_add(1, Ordering::Relaxed);
		}
		result
	}

	pub fn received_count(&self) -> u64 {
		self.received.load(Ordering::Relaxed)
	}
}

#[derive(Debug, Clone, Copy)]
pub struct LodLoadRequest {
	pub grid: GridId,
	pub requester: Entity,
	pub min: IVec3,
	pub size: IVec3,
	pub lod: f32,
	pub priority: f32,
	pub generation: u64,
}

#[derive(Resource)]
pub struct LodRequestChannel {
	sender: Sender<LodLoadRequest>,
	receiver: Receiver<LodLoadRequest>,
	sent: AtomicU64,
}

impl Default for LodRequestChannel {
	fn default() -> Self {
		let (sender, receiver) = unbounded();
		Self { sender, receiver, sent: AtomicU64::new(0) }
	}
}

impl LodRequestChannel {
	pub(crate) fn request(&self, request: LodLoadRequest) {
		if self.sender.send(request).is_ok() {
			self.sent.fetch_add(1, Ordering::Relaxed);
		}
	}

	pub fn receiver(&self) -> Receiver<LodLoadRequest> {
		self.receiver.clone()
	}

	pub fn try_recv(&self) -> Option<LodLoadRequest> {
		self.receiver.try_recv().ok()
	}

	pub fn sent_count(&self) -> u64 {
		self.sent.load(Ordering::Relaxed)
	}
}

#[derive(Debug, Clone)]
pub struct LodLoadResult {
	pub grid: GridId,
	pub requester: Entity,
	pub min: IVec3,
	pub size: IVec3,
	pub lod: f32,
	pub priority: f32,
	pub generation: u64,
	pub voxels: Option<Voxels>,
}

#[derive(Resource)]
pub struct LodLoaderChannel {
	sender: Sender<LodLoadResult>,
	receiver: Receiver<LodLoadResult>,
	received: AtomicU64,
}

impl Default for LodLoaderChannel {
	fn default() -> Self {
		let (sender, receiver) = unbounded();
		Self { sender, receiver, received: AtomicU64::new(0) }
	}
}

impl LodLoaderChannel {
	pub fn sender(&self) -> Sender<LodLoadResult> {
		self.sender.clone()
	}

	pub fn report(&self, result: LodLoadResult) {
		let _ = self.sender.send(result);
	}

	pub(crate) fn try_recv(&self) -> Option<LodLoadResult> {
		let result = self.receiver.try_recv().ok();
		if result.is_some() {
			self.received.fetch_add(1, Ordering::Relaxed);
		}
		result
	}

	pub fn received_count(&self) -> u64 {
		self.received.load(Ordering::Relaxed)
	}
}
