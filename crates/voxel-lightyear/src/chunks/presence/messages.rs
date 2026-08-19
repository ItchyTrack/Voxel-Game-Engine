use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use tile_data::NonZeroChunkRegion;
use voxel_data::grid::GridId;

use crate::chunks::request_id::NetworkRequestId;

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PresenceRequest {
	pub id: NetworkRequestId,
	pub grid: GridId,
}

impl MapEntities for PresenceRequest {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PresenceCancel {
	pub id: NetworkRequestId,
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PresenceLoad {
	pub id: NetworkRequestId,
	pub grid: GridId,
	pub region: NonZeroChunkRegion,
}

impl MapEntities for PresenceLoad {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PresenceLoaded {
	pub id: NetworkRequestId,
}
