use std::collections::HashSet;
use std::sync::{Arc, Mutex, OnceLock};

use bevy::prelude::*;
use tile_data::NonZeroChunkRegion;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::{ForgottenChunks, ChunkSource, RequestId, SourceCoverage, SourceHandle};
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
	) -> SourceCoverage {
		if cancellation.is_cancelled() || !self.state.remote_grids.lock().unwrap().contains(&grid) {
			return SourceCoverage::None;
		}
		let mut owned = 0_u32;
		for z in region.min().z..region.end().z {
			for y in region.min().y..region.end().y {
				for x in region.min().x..region.end().x {
					owned += (!self.state.forgotten.contains(grid, IVec3::new(x, y, z))) as u32;
				}
			}
		}
		if owned == 0 { return SourceCoverage::None; }
		self.state.loads.lock().unwrap().request_voxels(
			request_id,
			cancellation,
			grid,
			region,
			lod,
			voxel_type,
		);
		if owned == region.area() { SourceCoverage::All } else { SourceCoverage::Some }
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

	fn take_ownership(&self, grid: GridId, region: NonZeroChunkRegion) {
		for z in region.min().z..region.end().z {
			for y in region.min().y..region.end().y {
				for x in region.min().x..region.end().x {
					self.state.forgotten.forget(grid, IVec3::new(x, y, z));
				}
			}
		}
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
