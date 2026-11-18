use std::sync::atomic::{AtomicU64, Ordering};

use bevy::ecs::message::Message;
use bile_math::NonZeroRegion;
use voxel_data::{grid::GridId, voxels::Voxels};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RequestId(pub(crate) u64);

impl RequestId {
	pub(crate) fn next() -> Self {
		static NEXT_REQUEST_ID: AtomicU64 = AtomicU64::new(0);
		Self(NEXT_REQUEST_ID.fetch_add(1, Ordering::Relaxed))
	}

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
		voxels: Voxels,
	},
	VoxelsLoaded,
}

/// What a source sent for a request. Only requests made outside of Bile, through
/// [`crate::VoxelSources::request_voxels`], are published as messages.
#[derive(Debug, Message)]
pub struct SourceResult {
	pub request_id: RequestId,
	pub data: SourceResultData,
}
