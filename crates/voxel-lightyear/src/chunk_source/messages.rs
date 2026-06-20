use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::math::IVec3;
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use voxel_sources::LodKey;

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;

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
	pub voxels: Option<Voxels>,
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
	pub voxels: Option<Voxels>,
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
pub(crate) struct ChunkPresenceAabb {
	pub grid: GridId,
	pub min: IVec3,
	pub size: IVec3,
}

impl MapEntities for ChunkPresenceAabb {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}
