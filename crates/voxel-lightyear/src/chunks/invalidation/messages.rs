use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use voxel_data::grid::GridId;
use tile_data::ChunkRegion;

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RemoteChunkChangeKind {
	Changed { edit_index: u64 },
	Removed { edit_index: u64 },
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct RemoteChunkChanged {
	pub grid: GridId,
	pub region: ChunkRegion,
	pub kind: RemoteChunkChangeKind,
}

impl MapEntities for RemoteChunkChanged {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}
