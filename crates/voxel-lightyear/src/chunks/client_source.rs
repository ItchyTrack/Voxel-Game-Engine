use std::collections::{HashSet, VecDeque};
use std::sync::{Arc, Mutex, OnceLock};

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::{CancellationToken, ChunkSource, VoxelAreaKey, SourceHandle};

use super::presence::PresenceRequest;
use super::voxel_load::ClientLoadRegistry;
use crate::ReplicateVoxels;

const REMOTE_COST: u32 = 100;

#[derive(Default)]
pub(super) struct ClientChunkSourceState {
	pub handle: OnceLock<SourceHandle>,
	pub presence_requests: Mutex<VecDeque<PresenceRequest>>,
	pub loads: Mutex<ClientLoadRegistry>,
	pub remote_grids: Mutex<HashSet<GridId>>,
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

	fn cost(&self, grid: GridId, _chunk: IVec3) -> Option<u32> {
		self.state.remote_grids.lock().unwrap().contains(&grid).then_some(REMOTE_COST)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64, cancellation: CancellationToken) {
		if cancellation.is_cancelled() { return; }
		self.state.loads.lock().unwrap().request_chunk(grid, chunk, generation, cancellation);
	}

	fn cost_voxels(&self, grid: GridId, _min: IVec3, _size: IVec3, _lod: f32, _voxel_type: VoxelTypeId) -> Option<u32> {
		self.state.remote_grids.lock().unwrap().contains(&grid).then_some(REMOTE_COST)
	}

	fn request_voxel_area(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, voxel_type: VoxelTypeId, generation: u64, cancellation: CancellationToken) {
		if cancellation.is_cancelled() { return; }
		let key = VoxelAreaKey { min, size, lod: lod.max(0.0).floor() as u8 };
		self.state.loads.lock().unwrap().request_voxel_area(grid, key, voxel_type, 0.0, generation, cancellation);
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
