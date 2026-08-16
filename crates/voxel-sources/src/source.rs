use tile_data::NonZeroChunkRegion;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_tasks::CancellationToken;

use crate::request::RequestId;

use super::handle::SourceHandle;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceCoverage {
	/// The source owns none of the requested region and started no request.
	None,
	/// The source owns part of the region and started an asynchronous request.
	Some,
	/// The source owns the entire region and started an asynchronous request.
	All,
}

impl SourceCoverage {
	pub fn from_count(owned: usize, total: usize) -> Self {
		if owned == 0 { Self::None } else if owned == total { Self::All } else { Self::Some }
	}

	pub fn has_any(self) -> bool { self != Self::None }
}

pub trait ChunkSource: Send + Sync {
	fn init(&self, handle: SourceHandle);

	fn source_coverage(
		&self,
		grid: GridId,
		region: NonZeroChunkRegion,
	) -> SourceCoverage;

	fn request_voxels(
		&self,
		request_id: RequestId,
		cancellation: CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
	);

	fn request_presence(
		&self,
		request_id: RequestId,
		cancellation: CancellationToken,
		grid: GridId,
	);

	/// Relinquish ownership of the chunk immediately. (Existing requests must finish!)
	fn forget(&self, grid: GridId, region: NonZeroChunkRegion);
}
