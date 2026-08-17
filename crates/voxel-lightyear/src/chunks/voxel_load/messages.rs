use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::math::IVec3;
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_data::voxels_location::VoxelsLocation;
use voxel_sources::VoxelAreaKey;

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct VoxelLoadId(pub u64);

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub(crate) struct VoxelLoadRequest {
	pub id: VoxelLoadId,
	pub location: VoxelsLocation,
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
	pub id: VoxelLoadId,
	pub location: VoxelsLocation,
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
	pub id: VoxelLoadId,
	pub outcome: VoxelLoadOutcome,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum VoxelLoadOutcome {
	Cancelled,
	Received,
}
