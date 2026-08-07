use std::sync::Arc;

use bevy::prelude::*;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_tasks::CancellationToken;

use super::handle::SourceHandle;

pub(super) type SharedSource = Arc<dyn ChunkSource>;

/// A source of voxel chunks. Methods take `&self` and are called concurrently from the
/// async load workers, so mutable state must use interior mutability. This lets many
/// loads from one source run in parallel instead of serializing on an outer lock.
pub trait ChunkSource: Send + Sync {
	fn init(&self, handle: SourceHandle);

	/// Cost of serving `chunk`, or `None` if this source can't. Lowest wins.
	fn cost(&self, grid: GridId, chunk: IVec3) -> Option<u32>;

	/// Start loading a chunk. Expensive implementations should check
	/// `cancellation` periodically and return without publishing when cancelled.
	fn request_load(
		&self,
		grid: GridId,
		chunk: IVec3,
		generation: u64,
		cancellation: CancellationToken,
	);

	fn cost_tile_voxels(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, voxel_type: VoxelTypeId) -> Option<u32>;

	fn request_tile_voxels(
		&self,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		generation: u64,
		cancellation: CancellationToken,
	);

	/// Publish any claimed areas, then call [`SourceHandle::presence_loaded`] exactly once.
	fn request_available_area(&self, grid: GridId);

	fn save(&self, grid: GridId, chunk: IVec3, voxels: &Voxels) -> bool {
		let _ = (grid, chunk, voxels);
		false
	}

	fn forget(&self, grid: GridId, chunk: IVec3) {
		let _ = (grid, chunk);
	}
}
