use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use tile_data::NonZeroChunkRegion;
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::edit::GridGeneration;

use crate::chunks::request_id::NetworkRequestId;

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct VoxelRequestKey {
	pub grid: GridId,
	pub region: NonZeroChunkRegion,
	pub lod: u8,
	pub voxel_type: Option<VoxelTypeId>,
	pub generation: GridGeneration,
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct VoxelLoadRequest {
	pub id: NetworkRequestId,
	pub key: VoxelRequestKey,
}

impl MapEntities for VoxelLoadRequest {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.key.grid = entity_mapper.get_mapped(self.key.grid);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct VoxelLoadCancel {
	pub id: NetworkRequestId,
}

#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub(crate) struct VoxelLoadPayload {
	pub id: NetworkRequestId,
	pub grid: GridId,
	pub region: NonZeroChunkRegion,
	pub lod: u8,
	pub generation: GridGeneration,
	pub voxels: CompressedVoxels,
}

impl MapEntities for VoxelLoadPayload {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.grid = entity_mapper.get_mapped(self.grid);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct VoxelLoadComplete {
	pub id: NetworkRequestId,
	pub sent_payload_count: u32,
}
