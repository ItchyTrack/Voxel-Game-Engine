use std::collections::HashSet;
use std::sync::{Arc, Mutex, OnceLock};

use bevy::prelude::*;
use tile_data::NonZeroChunkRegion;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::{ForgottenChunks, ChunkSource, RequestId, SourceCoverage, SourceHandle, edit::GridGeneration};
use voxel_tasks::CancellationToken;

use super::presence::ClientPresenceRegistry;
use super::voxel_load::ClientLoadRegistry;
use crate::ReplicateVoxels;

#[derive(Default)]
pub(super) struct ClientChunkSourceState {
	pub handle: OnceLock<SourceHandle>,
	pub presence: Mutex<ClientPresenceRegistry>,
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
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
		generation: GridGeneration,
	) -> SourceCoverage {
		if cancellation.is_cancelled() || !self.state.remote_grids.lock().unwrap().contains(&grid) {
			return SourceCoverage::None;
		}
		let mut accepted_chunks = HashSet::new();
		for z in region.min().z..region.end().z {
			for y in region.min().y..region.end().y {
				for x in region.min().x..region.end().x {
					let chunk = IVec3::new(x, y, z);
					if !self.state.forgotten.contains(grid, chunk) {
						accepted_chunks.insert(chunk);
					}
				}
			}
		}
		if accepted_chunks.is_empty() { return SourceCoverage::None; }
		let coverage = if accepted_chunks.len() == region.area() as usize { SourceCoverage::All } else { SourceCoverage::Some };
		self.state.loads.lock().unwrap().request_voxels(
			request_id,
			cancellation,
			grid,
			region,
			lod,
			voxel_type,
			generation,
			accepted_chunks,
		);
		coverage
	}

	fn request_presence(
		&self,
		request_id: RequestId,
		cancellation: CancellationToken,
		grid: GridId,
	) {
		if cancellation.is_cancelled() { return; }
		if self.state.remote_grids.lock().unwrap().contains(&grid) {
			self.state.presence.lock().unwrap().request(request_id, cancellation, grid);
		} else if let Some(handle) = self.state.handle.get() {
			handle.presence_loaded(request_id);
		}
	}

	fn acquire_ownership(&self, grid: GridId, region: NonZeroChunkRegion) {
		self.state.forgotten.remember_area(grid, region);
	}

	fn relinquish_ownership(&self, grid: GridId, region: NonZeroChunkRegion) {
		self.state.forgotten.forget_area(grid, region);
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
