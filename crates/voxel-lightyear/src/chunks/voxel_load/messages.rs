use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::math::IVec3;
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use tile_data::NonZeroChunkRegion;
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::RequestId;

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub(crate) struct VoxelLoadRequest {
	pub request_id: RequestId,
	pub grid: GridId,
	pub region: NonZeroVoxelRegion,
	pub required_lod: u8, // voxel_size = 2^scale
	pub voxel_type: Option<VoxelTypeId>,
	pub priority: f32,
}

impl MapEntities for VoxelLoadRequest {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.location.grid_id = entity_mapper.get_mapped(self.location.grid_id);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub(crate) struct VoxelLoadResponse {
	pub request_id: RequestId,
	pub grid: GridId,
	pub region: NonZeroChunkRegion,
	pub lod: u8,
	pub generation: u64,
	pub voxels: Option<CompressedVoxels>,
}

impl MapEntities for VoxelLoadResponse {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		self.location.grid_id = entity_mapper.get_mapped(self.location.grid_id);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct VoxelLoadFinished {
	pub request_id: RequestId,
	pub outcome: VoxelLoadOutcome,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum VoxelLoadOutcome {
	Cancelled,
	Received,
}
