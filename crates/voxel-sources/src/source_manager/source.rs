use std::sync::Arc;

use bevy::prelude::*;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_tasks::CancellationToken;

use super::handle::SourceHandle;

pub(super) type SharedSource = Arc<dyn ChunkSource>;

pub enum LendResult {
	Unavailable,
	Borrowed(Option<Voxels>),
}

/// A source of voxel chunks. Sources own their ownership and lending state;
/// SourceManager only coordinates transfers and routes requests through the
/// sources that currently offer data.
pub trait ChunkSource: Send + Sync {
	fn init(&self, handle: SourceHandle);

	/// Cost of serving `chunk`, or `None` when this source does not currently
	/// serve it (including while an owned chunk is lent to another source).
	fn cost(&self, grid: GridId, chunk: IVec3) -> Option<u32>;

	fn request_load(
		&self,
		grid: GridId,
		chunk: IVec3,
		edit_index: u64,
		cancellation: CancellationToken,
	) -> bool;

	fn cost_voxels(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, voxel_type: VoxelTypeId) -> Option<u32>;

	fn request_voxel_area(
		&self,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		edit_index: u64,
		cancellation: CancellationToken,
	) -> bool;

	fn request_available_area(&self, grid: GridId);

	/// Retain the owned data for crash safety, stop serving it, and pass a copy
	/// to a borrower. Called on a source worker.
	fn lend(&self, grid: GridId, chunk: IVec3, cancellation: CancellationToken) -> LendResult;

	/// Install data borrowed from another source and become its serving source.
	fn accept_borrow(&self, grid: GridId, chunk: IVec3, voxels: Option<Voxels>) -> bool {
		let _ = (grid, chunk, voxels);
		false
	}

	/// Create and own a chunk confirmed absent from every source.
	fn create_owned(&self, grid: GridId, chunk: IVec3) -> bool {
		let _ = (grid, chunk);
		false
	}

	/// Return every borrow/lend in a chunk-space area. Sources resolve this from
	/// their own internal state; newly owned chunks remain owned.
	fn return_area(&self, grid: GridId, min: IVec3, size: IVec3);

	/// Persist data at an already-assigned edit index. Saving transfers ownership
	/// but does not edit the chunk and does not publish an edit event.
	fn save(&self, grid: GridId, chunk: IVec3, edit_index: u64, voxels: &Voxels) -> bool {
		let _ = (grid, chunk, edit_index, voxels);
		false
	}

	/// Remove either owned or borrowed state for the chunk.
	fn forget(&self, grid: GridId, chunk: IVec3);
}
