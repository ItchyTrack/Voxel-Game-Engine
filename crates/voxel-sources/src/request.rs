use bevy::ecs::message::Message;
use tile_data::NonZeroChunkRegion;
use voxel_data::{grid::GridId, voxels::Voxels};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RequestId(pub(crate) u64);

#[derive(Debug)]
pub enum SourceResultData {
	Presence {
		grid: GridId,
		region: NonZeroChunkRegion,
	},
	PresenceLoaded,
	Voxels {
		grid: GridId,
		region: NonZeroChunkRegion,
		lod: u8,
		generation: u64,
		voxels: Voxels,
	},
	VoxelsLoaded,
}

#[derive(Debug, Message)]
pub struct SourceResult {
	pub request_id: RequestId,
	pub data: SourceResultData,
}
