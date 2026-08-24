use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use voxel_data::grid::GridId;
use tile_data::ChunkRegion;
use voxel_sources::edit::{GridGeneration, GridEditId};

#[derive(Event, Clone, Copy, Debug, Serialize, Deserialize)]
pub(super) struct EditInterest {
	pub grid: GridId,
	pub region: ChunkRegion,
	pub version: u64,
	pub interested: bool,
}

impl MapEntities for EditInterest {
	fn map_entities<M: EntityMapper>(&mut self, mapper: &mut M) { self.grid = mapper.get_mapped(self.grid); }
}

#[derive(Event, Clone, Copy, Debug, Serialize, Deserialize)]
pub(super) struct EditStreamStart {
	pub grid: GridId,
	pub first_stream_sequence: u64,
}

impl MapEntities for EditStreamStart {
	fn map_entities<M: EntityMapper>(&mut self, mapper: &mut M) { self.grid = mapper.get_mapped(self.grid); }
}

#[derive(Event, Clone, Debug, Serialize, Deserialize)]
pub(super) struct RemoteGridEdit {
	pub grid: GridId,
	pub grid_edit_id: GridEditId,
	pub generation: GridGeneration,
	pub edit: Box<[u8]>,
}

impl MapEntities for RemoteGridEdit {
	fn map_entities<M: EntityMapper>(&mut self, mapper: &mut M) { self.grid = mapper.get_mapped(self.grid); }
}
