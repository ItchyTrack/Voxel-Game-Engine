use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use tile_data::NonZeroChunkRegion;
use voxel_data::grid::GridId;
use voxel_sources::edit::GridEditId;

#[derive(Event, Clone, Copy, Debug, Serialize, Deserialize)]
pub(super) struct EditInterest {
	pub grid: GridId,
	pub region: NonZeroChunkRegion,
	pub interested: bool,
}

impl MapEntities for EditInterest {
	fn map_entities<M: EntityMapper>(&mut self, mapper: &mut M) { self.grid = mapper.get_mapped(self.grid); }
}

#[derive(Event, Clone, Debug, Serialize, Deserialize)]
pub(super) struct RemoteGridEdit {
	pub grid: GridId,
	pub edit_id: GridEditId,
	pub edit: Box<[u8]>,
}

impl MapEntities for RemoteGridEdit {
	fn map_entities<M: EntityMapper>(&mut self, mapper: &mut M) { self.grid = mapper.get_mapped(self.grid); }
}
