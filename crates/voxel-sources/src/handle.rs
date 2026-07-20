use bevy::math::IVec3;
use crossbeam_channel::Sender;

use std::sync::Arc;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};

use crate::source::{SourceId, VoxelLodGenerator, VoxelLodGenerators};
use crate::ChunkPresenceLoaded;

#[derive(bevy::ecs::message::Message, Debug, Clone, Copy)]
pub enum SourceEvent {
	Claim { source: SourceId, grid: GridId, min: IVec3, size: IVec3 },
	Unavailable { grid: GridId, min: IVec3, size: IVec3 },
}

pub struct SourceChunkResult {
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
	Chunk(SourceChunkResult),
	Lod(SourceLodResult),
}

#[derive(Clone)]
pub struct SourceHandle {
	pub(crate) id: SourceId,
	pub(crate) messages: Sender<SourceMessage>,
	pub(crate) lod_generators: VoxelLodGenerators,
}

impl SourceHandle {
	pub fn id(&self) -> SourceId {
		self.id
	}

	pub fn voxel_lod_generator(&self, type_id: VoxelTypeId) -> Option<Arc<dyn VoxelLodGenerator>> {
		self.lod_generators.read().unwrap().get(&type_id).cloned()
	}

	pub fn loaded(&self, grid: GridId, chunk: IVec3, generation: u64, voxels: Option<Voxels>) {
		let _ = self.messages.send(SourceMessage::Chunk(SourceChunkResult { grid, chunk, generation, voxels }));
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
