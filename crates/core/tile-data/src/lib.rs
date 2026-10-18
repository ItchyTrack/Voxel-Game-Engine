pub mod chunk;
mod building;
mod voxel_reducer;

pub use chunk::{CHUNK_SIZE, chunk_aligned, chunk_of, chunk_origin, chunk_region, chunks_in};
pub use building::{
	async_trait, TileBuildingVoxelReader, ReceiveVoxelsFuture, VoxelRegionRequest,
	VoxelRegionResult, TileBuildingSession, TileBuilder,
};
pub use voxel_reducer::{TileVoxelReducer, TileVoxelReducerRegistry};

use bevy::prelude::*;

pub trait TileAppExt {
	fn register_tile_voxel_reducer<R: TileVoxelReducer>(&mut self, reducer: R) -> &mut Self;
}

impl TileAppExt for App {
	fn register_tile_voxel_reducer<R: TileVoxelReducer>(&mut self, reducer: R) -> &mut Self {
		self.init_resource::<TileVoxelReducerRegistry>();
		self.world().resource::<TileVoxelReducerRegistry>().insert(reducer);
		self
	}
}
