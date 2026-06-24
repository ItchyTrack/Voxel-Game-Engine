use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::math::IVec3;
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use voxel_sources::LodKey;

use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::grid::GridId;

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ChunkRequest {
	pub grid: GridId,
	pub chunk: IVec3,
}

impl MapEntities for ChunkRequest {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub(crate) struct ChunkResponse {
	pub grid: GridId,
	pub chunk: IVec3,
	pub generation: u64,
	pub voxels: Option<CompressedVoxels>,
}

impl MapEntities for ChunkResponse {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub(crate) struct LodRequest {
	pub grid: GridId,
	pub key: LodKey,
	pub priority: f32,
}

impl MapEntities for LodRequest {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub(crate) struct LodResponse {
	pub grid: GridId,
	pub key: LodKey,
	pub generation: u64,
	pub voxels: Option<CompressedVoxels>,
}

impl MapEntities for LodResponse {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PresenceRequest {
	pub grid: GridId,
}

impl MapEntities for PresenceRequest {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PresenceLoad {
	pub grid: GridId,
	pub area: Option<(IVec3, IVec3)>,
}

impl MapEntities for PresenceLoad {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RemoteChunkChangeKind {
	Changed { generation: u64 },
	Removed { generation: u64 },
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct RemoteChunkChanged {
	pub grid: GridId,
	pub min: IVec3,
	pub size: IVec3,
	pub kind: RemoteChunkChangeKind,
}

impl MapEntities for RemoteChunkChanged {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}
