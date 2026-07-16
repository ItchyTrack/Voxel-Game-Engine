use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::math::IVec3;
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use voxel_data::grid::GridId;

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
pub(super) struct PresenceLoad {
	pub grid: GridId,
	pub area: Option<(IVec3, IVec3)>,
}

impl MapEntities for PresenceLoad {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}
