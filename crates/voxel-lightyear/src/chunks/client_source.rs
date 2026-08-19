use std::collections::{HashSet, VecDeque};
use std::sync::{Arc, Mutex, OnceLock};

use bevy::prelude::*;
use tile_data::{ChunkRegion, NonZeroChunkRegion};
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::{CancellationToken, ChunkSource, SourceCoverage, SourceHandle};

use voxel_streaming::ForgottenChunks;

use super::presence::PresenceRequest;
use super::voxel_load::ClientLoadRegistry;
use crate::ReplicateVoxels;

#[derive(Default)]
pub(super) struct ClientChunkSourceState {
	pub handle: OnceLock<SourceHandle>,
	pub presence_requests: Mutex<VecDeque<PresenceRequest>>,
	pub loads: Mutex<ClientLoadRegistry>,
	pub remote_grids: Mutex<HashSet<GridId>>,
	pub forgotten: ForgottenChunks,
}

#[derive(Clone, Default, Resource)]
pub struct ClientChunkSource {
	pub(super) state: Arc<ClientChunkSourceState>,
}

impl ChunkSource for ClientChunkSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.state.handle.set(handle);
	}

	fn request_voxels(
		&self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		required_lod: u8,
		voxel_type: Option<VoxelTypeId>,
	) -> SourceCoverage {
		if !self.state.remote_grids.lock().unwrap().contains(&grid) { return SourceCoverage::None; }
		let mut has_one = None;
		let mut source_coverage = SourceCoverage::All;
		'outer:for x in region.min().x..region.end().x {
			for y in region.min().y..region.end().y {
				for z in region.min().z..region.end().z {
					if store.contains_chunk(IVec3::new(x, y, z)) {
						if let Some(has_one_val) = has_one {
							if has_one_val {
								source_coverage = SourceCoverage::Some;
								break 'outer;
							}
						} else {
							has_one = Some(true)
						}
					} else {
						if let Some(has_one_val) = has_one {
							if has_one_val {
								source_coverage = SourceCoverage::Some;
								break 'outer;
							}
						} else {
							has_one = Some(false)
						}
					}
				}
			}
		}
		assert!(has_one.is_some()); // has to happen as region is non zero
		if !has_one.unwrap() {
			return SourceCoverage::None;
		}
		self.state.loads.lock().unwrap().request_voxel_area(grid, regionkey, voxel_type, 0.0, generation, cancellation);
		source_coverage
	}

	fn request_presence(
		&self,
		request_id: RequestId,
		cancellation: CancellationToken,
		grid: GridId,
	) {
		todo!()
	}

	fn take_ownership(
		&self,
		grid: GridId,
		region: NonZeroChunkRegion
	) {
		self.state.forgotten.forget(grid, chunk);
	}
}

pub(super) fn register_remote_voxel_grids(
	source: Res<ClientChunkSource>,
	grids: Query<GridId, With<ReplicateVoxels>>,
) {
	let mut remote_grids = source.state.remote_grids.lock().unwrap();
	for grid in &grids {
		remote_grids.insert(grid);
	}
}
