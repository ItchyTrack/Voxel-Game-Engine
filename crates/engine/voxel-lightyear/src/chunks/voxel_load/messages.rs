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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub(crate) struct VoxelPayloadIndex(pub(crate) u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct VoxelPayloadMetadata {
	pub region: NonZeroChunkRegion,
	pub lod: u8,
	pub compressed_bytes: u64,
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

#[derive(Event, Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
pub(crate) struct VoxelLoadManifest {
	pub id: NetworkRequestId,
	pub payloads: Box<[VoxelPayloadMetadata]>,
}

#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub(crate) struct VoxelLoadPayload {
	pub id: NetworkRequestId,
	pub index: VoxelPayloadIndex,
	pub voxels: CompressedVoxels,
}

#[derive(Event, Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
pub(crate) struct VoxelLoadRetry {
	pub id: NetworkRequestId,
	pub missing: Box<[VoxelPayloadIndex]>,
}

#[derive(Event, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct VoxelLoadReceived {
	pub id: NetworkRequestId,
}
