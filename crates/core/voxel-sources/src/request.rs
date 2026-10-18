use bevy::ecs::message::Message;
use bile_math::NonZeroRegion;
use voxel_data::{grid::GridId, voxels::Voxels};

use crate::edit::GridGeneration;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RequestId(pub(crate) u64);

// #[cfg(test)] // TODO: figure out why this was not working
impl RequestId {
	pub fn from_raw(request_id: u64) -> RequestId {
		RequestId(request_id)
	}
}

#[derive(Debug)]
pub enum SourceResultData {
	Presence {
		grid: GridId,
		region: NonZeroRegion,
	},
	PresenceLoaded,
	Voxels {
		grid: GridId,
		region: NonZeroRegion,
		lod: u8,
		generation: GridGeneration,
		voxels: Voxels,
	},
	VoxelsLoaded {
		generation: GridGeneration,
	},
}

#[derive(Debug, Message)]
pub struct SourceResult {
	pub request_id: RequestId,
	pub data: SourceResultData,
}
