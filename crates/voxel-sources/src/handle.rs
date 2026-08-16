use crossbeam_channel::Sender;
use tile_data::NonZeroChunkRegion;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};

use crate::request::{RequestId, SourceResult, SourceResultData};

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
		grid: GridId,
		region: NonZeroChunkRegion,
		lod: u8,
		voxel_type: VoxelTypeId,
		generation: u64,
		voxels: Option<Voxels>,
		request_id: RequestId,
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

	pub fn presence(&self, grid: GridId, region: NonZeroChunkRegion, request_id: RequestId) {
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
