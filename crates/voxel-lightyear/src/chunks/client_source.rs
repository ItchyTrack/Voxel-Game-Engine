use std::collections::{HashSet, VecDeque};
use std::sync::{Arc, Mutex, OnceLock};

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::{CancellationToken, ChunkSource, LendResult, LentChunks, VoxelAreaKey, SourceHandle};

use voxel_streaming::ForgottenChunks;

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
	pub forgotten: ForgottenChunks,
	pub lent: LentChunks,
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

	fn cost(&self, grid: GridId, chunk: IVec3) -> Option<u32> {
		if self.state.forgotten.contains(grid, chunk) || self.state.lent.contains(grid, chunk) { return None; }
		self.state.remote_grids.lock().unwrap().contains(&grid).then_some(REMOTE_COST)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, edit_index: u64, cancellation: CancellationToken) -> bool {
		if cancellation.is_cancelled() || self.state.forgotten.contains(grid, chunk) || self.state.lent.contains(grid, chunk) { return false; }
		self.state.loads.lock().unwrap().request_chunk(grid, chunk, edit_index, cancellation);
		true
	}

	fn cost_voxels(&self, grid: GridId, min: IVec3, size: IVec3, _lod: f32, _voxel_type: VoxelTypeId) -> Option<u32> {
		if !self.state.forgotten.any_remembered_in(grid, min, size) || !self.state.lent.any_available_in(grid, min, size) { return None; }
		self.state.remote_grids.lock().unwrap().contains(&grid).then_some(REMOTE_COST)
	}

	fn request_voxel_area(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, voxel_type: VoxelTypeId, edit_index: u64, cancellation: CancellationToken) -> bool {
		if cancellation.is_cancelled() || !self.state.lent.any_available_in(grid, min, size) { return false; }
		let key = VoxelAreaKey::new(min, size.as_uvec3(), lod.max(0.0).floor() as u8);
		self.state.loads.lock().unwrap().request_voxel_area(grid, key, voxel_type, 0.0, edit_index, cancellation);
		true
	}

	fn lend(&self, grid: GridId, chunk: IVec3, cancellation: CancellationToken) -> LendResult {
		if self.state.forgotten.contains(grid, chunk) || !self.state.lent.begin(grid, chunk) { return LendResult::Unavailable; }
		let Some(handle) = self.state.handle.get() else {
			self.state.lent.end(grid, chunk);
			return LendResult::Unavailable;
		};
		let received = self.state.loads.lock().unwrap().request_lend(grid, chunk, handle.current_edit_index(grid), cancellation);
		match received.recv() {
			Ok(result) => result,
			Err(_) => {
				self.state.lent.end(grid, chunk);
				LendResult::Unavailable
			}
		}
	}


	fn return_area(&self, grid: GridId, min: IVec3, size: IVec3) { self.state.lent.end_area(grid, min, size); }

	fn forget(&self, grid: GridId, chunk: IVec3) {
		self.state.forgotten.forget(grid, chunk);
		self.state.lent.end(grid, chunk);
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
