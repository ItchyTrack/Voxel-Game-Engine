use crossbeam_channel::Sender;
use tile_data::NonZeroChunkRegion;
use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_tasks::CancellationToken;

use crate::SourceResult;
use crate::request::{RequestId, SourceResultData};

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct SourceId(pub usize);

#[derive(Clone)]
pub struct SourceHandle {
	pub(super) id: SourceId,
	pub(super) messages: Sender<SourceResult>,
}

impl SourceHandle {
	pub fn id(&self) -> SourceId { self.id }

	pub fn voxels(
		&self,
		request_id: RequestId,
		location: voxels_location,
		generation: u64,
		voxels: Voxels,
	) {
		let _ = self.messages.send(SourceResult {
			request_id,
			data: SourceResultData::Voxels {
				grid,
				region,
				lod,
				voxel_type,
				generation,
				voxels,
			},
		});
	}

	pub fn voxels_loaded(&self, request_id: RequestId) {
		let _ = self.messages.send(SourceResult {
			request_id,
			data: SourceResultData::VoxelsLoaded,
		});
	}

	pub fn presence(&self, request_id: RequestId, grid: GridId, region: NonZeroChunkRegion) {
		let _ = self.messages.send(SourceResult {
			request_id,
			data: SourceResultData::Presence{ grid, region },
		});
	}

	pub fn presence_loaded(&self, request_id: RequestId) {
		let _ = self.messages.send(SourceResult {
			request_id,
			data: SourceResultData::PresenceLoaded,
		});
	}
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceCoverage {
	/// The source owns part of the region and started an asynchronous request.
	Some,
	/// The source owns the entire region and started an asynchronous request.
	All,
}

pub trait ChunkSource: Send + Sync {
	fn init(&self, handle: SourceHandle);

	fn request_voxels(
		&self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
	) -> Option<(SourceCoverage, Option<impl AsyncFnOnce() + Send + 'static>)>;

	fn request_presence(
		&self,
		request_id: RequestId,
		cancellation: CancellationToken,
		grid: GridId,
	);

	fn take_ownership(
		&self,
		grid: GridId,
		region: NonZeroChunkRegion
	);
}
