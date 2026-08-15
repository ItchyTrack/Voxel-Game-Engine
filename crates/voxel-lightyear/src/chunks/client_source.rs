use std::collections::{HashSet, VecDeque};
use std::sync::{Arc, Mutex, OnceLock};

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::{CancellationToken, ChunkSource, SourceCoverage, TakeJob, VoxelAreaKey, SourceHandle};

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

	fn request_available_area(&self, grid: GridId) {
		self.state.presence_requests.lock().unwrap().push_back(PresenceRequest { grid });
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64, cancellation: CancellationToken) -> SourceCoverage {
		if cancellation.is_cancelled()
			|| self.state.forgotten.contains(grid, chunk)
			|| !self.state.remote_grids.lock().unwrap().contains(&grid) { return SourceCoverage::None; }
		self.state.loads.lock().unwrap().request_chunk(grid, chunk, generation, None, cancellation);
		SourceCoverage::All
	}

	fn request_voxel_area(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, voxel_type: VoxelTypeId, generation: u64, cancellation: CancellationToken) -> SourceCoverage {
		if cancellation.is_cancelled() || !self.state.remote_grids.lock().unwrap().contains(&grid) { return SourceCoverage::None; }
		let mut owned = 0;
		for z in 0..size.z { for y in 0..size.y { for x in 0..size.x {
			owned += (!self.state.forgotten.contains(grid, min + IVec3::new(x, y, z))) as usize;
		}}}
		let coverage = SourceCoverage::from_count(owned, (size.x * size.y * size.z) as usize);
		if coverage == SourceCoverage::None { return coverage; }
		let key = VoxelAreaKey::new(min, size.as_uvec3(), lod.max(0.0).floor() as u8);
		self.state.loads.lock().unwrap().request_voxel_area(grid, key, voxel_type, 0.0, generation, cancellation);
		coverage
	}

	fn take(&self, destination: voxel_sources::SourceId, grid: GridId, min: IVec3, size: IVec3, generation: u64) -> Vec<TakeJob> {
		if !self.state.remote_grids.lock().unwrap().contains(&grid) { return Vec::new(); }
		let chunks = self.state.forgotten.forget_area_where(grid, min, size, |_| true);
		let source = self.clone();
		chunks.into_iter().map(|chunk| TakeJob::new(chunk, {
			let source = source.clone();
			move || {
				source.state.loads.lock().unwrap().request_chunk(
					grid,
					chunk,
					generation,
					Some(destination),
					CancellationToken::new(),
				);
			}
		})).collect()
	}

	fn forget(&self, grid: GridId, chunk: IVec3) {
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
