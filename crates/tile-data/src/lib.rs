mod capability_registry;
pub mod chunk;
mod class;
mod data;
mod building;
mod index;
mod key;
mod voxel_reducer;

pub use capability_registry::TileCapabilityRegistry;
pub use chunk::{
	CHUNK_SIZE, ChunkRegion, NonZeroChunkRegion, chunk_of, chunk_origin,
	chunks_covering_nonzero_voxel_region, chunks_covering_voxel_region,
	nonzero_voxel_region_from_chunks, voxel_region_from_chunks,
};
pub use class::{TileClassId, TileClassRegistry, TileBuildingParameters};
pub use data::{DynamicTileData, LoadedTile, TileData};
pub use building::{
	async_trait, TileBuildingVoxelReader, ReceiveVoxelsFuture, VoxelRegionRequest,
	VoxelRegionResult, TileBuildingSession, TileBuilder, TileBuilderRegistry,
};
pub use index::{TileIndex, TileIndexKey};
pub use key::TileKey;
pub use voxel_reducer::{TileVoxelReducer, TileVoxelReducerRegistry};

use bevy::prelude::*;

#[derive(Default)]
pub struct TileDataPlugin;

impl Plugin for TileDataPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<TileClassRegistry>()
			.init_resource::<TileBuilderRegistry>();
	}
}

pub trait TileAppExt {
	fn register_tile_class(&mut self) -> TileClassId;
	fn register_tile_builder<G: TileBuilder>(&mut self, tile_class_id: TileClassId, generator: G) -> &mut Self;
	fn register_tile_voxel_reducer<R: TileVoxelReducer>(&mut self, reducer: R) -> &mut Self;
}

impl TileAppExt for App {
	fn register_tile_class(&mut self) -> TileClassId {
		self.init_resource::<TileClassRegistry>();
		self.world_mut().resource_mut::<TileClassRegistry>().register()
	}

	fn register_tile_builder<G: TileBuilder>(&mut self, tile_class_id: TileClassId, generator: G) -> &mut Self {
		self.init_resource::<TileBuilderRegistry>();
		self.world_mut().resource_mut::<TileBuilderRegistry>().insert(tile_class_id, generator);
		self
	}

	fn register_tile_voxel_reducer<R: TileVoxelReducer>(&mut self, reducer: R) -> &mut Self {
		self.init_resource::<TileVoxelReducerRegistry>();
		self.world().resource::<TileVoxelReducerRegistry>().insert(reducer);
		self
	}
}
