use bevy::math::IVec3;
use crossbeam_channel::Sender;

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;

use crate::source::SourceId;
use crate::ChunkPresenceLoaded;

#[derive(bevy::ecs::message::Message, Debug, Clone, Copy)]
pub enum SourceEvent {
	Claim { source: SourceId, grid: GridId, min: IVec3, size: IVec3 },
	Unavailable { grid: GridId, min: IVec3, size: IVec3 },
}

pub struct SourceResult {
	pub grid: GridId,
	pub chunk: IVec3,
	pub generation: u64,
	pub voxels: Option<Voxels>,
}

pub struct SourceLodResult {
	pub source: SourceId,
	pub grid: GridId,
	pub min: IVec3,
	pub size: IVec3,
	pub lod: f32,
	pub generation: u64,
	pub voxels: Option<Voxels>,
}

pub enum SourceMessage {
	Event(SourceEvent),
	ChunkChanged(crate::ChunkChanged),
	PresenceLoaded(ChunkPresenceLoaded),
	Chunk(SourceResult),
	Lod(SourceLodResult),
}

#[derive(Clone)]
pub struct SourceHandle {
	pub(crate) id: SourceId,
	pub(crate) messages: Sender<SourceMessage>,
}

impl SourceHandle {
	pub fn id(&self) -> SourceId {
		self.id
	}

	/// Finished a load. `None` voxels = confirmed empty.
	pub fn loaded(&self, grid: GridId, chunk: IVec3, generation: u64, voxels: Option<Voxels>) {
		let _ = self.messages.send(SourceMessage::Chunk(SourceResult { grid, chunk, generation, voxels }));
	}

	pub fn loaded_lod(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, generation: u64, voxels: Option<Voxels>) {
		let _ = self.messages.send(SourceMessage::Lod(SourceLodResult { source: self.id, grid, min, size, lod, generation, voxels }));
	}

	pub fn claim(&self, grid: GridId, min: IVec3, size: IVec3) {
		let _ = self.messages.send(SourceMessage::Event(SourceEvent::Claim { source: self.id, grid, min, size }));
	}

	pub fn unavailable(&self, grid: GridId, min: IVec3, size: IVec3) {
		let _ = self.messages.send(SourceMessage::Event(SourceEvent::Unavailable { grid, min, size }));
	}

	pub fn presence_loaded(&self, grid: GridId) {
		let _ = self.messages.send(SourceMessage::PresenceLoaded(ChunkPresenceLoaded { grid }));
	}
}
