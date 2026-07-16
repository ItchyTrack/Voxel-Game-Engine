use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::math::IVec3;
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use voxel_data::grid::GridId;

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
