use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::math::IVec3;
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::VoxelAreaKey;

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct VoxelLoadId(pub u64);

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub(crate) struct VoxelLoadRequest {
	pub id: VoxelLoadId,
	pub kind: VoxelLoadRequestKind,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub(crate) enum VoxelLoadRequestKind {
	Chunk { grid: GridId, chunk: IVec3 },
	VoxelArea { grid: GridId, key: VoxelAreaKey, voxel_type: VoxelTypeId, priority: f32 },
}

impl MapEntities for VoxelLoadRequest {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		let grid = match &mut self.kind {
			VoxelLoadRequestKind::Chunk { grid, .. } | VoxelLoadRequestKind::VoxelArea { grid, .. } => grid,
		};
		*grid = entity_mapper.get_mapped(*grid);
	}
}

#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub(crate) struct VoxelLoadResponse {
	pub id: VoxelLoadId,
	pub kind: VoxelLoadResponseKind,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub(crate) enum VoxelLoadResponseKind {
	Chunk {
		grid: GridId,
		chunk: IVec3,
		generation: u64,
		voxels: Option<CompressedVoxels>,
	},
	VoxelArea {
		grid: GridId,
		key: VoxelAreaKey,
		generation: u64,
		voxel_type: VoxelTypeId,
		voxels: Option<CompressedVoxels>,
	},
}

impl MapEntities for VoxelLoadResponse {
	fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
		let grid = match &mut self.kind {
			VoxelLoadResponseKind::Chunk { grid, .. } | VoxelLoadResponseKind::VoxelArea { grid, .. } => grid,
		};
		*grid = entity_mapper.get_mapped(*grid);
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
