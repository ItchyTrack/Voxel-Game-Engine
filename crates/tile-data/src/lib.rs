mod capability_registry;
pub mod chunk;
mod class;
mod data;
mod generator;
mod index;
mod key;

pub use capability_registry::TileCapabilityRegistry;
pub use chunk::{
	CHUNK_SIZE, ChunkRegion, NonZeroChunkRegion, chunk_of, chunk_origin,
	chunks_covering_nonzero_voxel_region, chunks_covering_voxel_region,
	nonzero_voxel_region_from_chunks, voxel_region_from_chunks,
};
pub use class::{TileClassId, TileClassRegistry, TileGenerationParameters};
pub use data::{DynamicTileData, LoadedTile, TileData};
pub use generator::{
	async_trait, GenerationVoxelReader, ReceiveVoxelsFuture,
	TileGenerationSession, TileGenerator, TileGeneratorRegistry,
};
pub use index::{TileIndex, TileIndexKey};
pub use key::TileKey;

use bevy::prelude::*;

pub trait TileAppExt {
	fn register_tile_class(&mut self) -> TileClassId;
	fn register_tile_generator<G: TileGenerator>(&mut self, tile_class_id: TileClassId, generator: G) -> &mut Self;
}

impl TileAppExt for App {
	fn register_tile_class(&mut self) -> TileClassId {
		self.init_resource::<TileClassRegistry>();
		self.world_mut().resource_mut::<TileClassRegistry>().register()
	}

	fn register_tile_generator<G: TileGenerator>(&mut self, tile_class_id: TileClassId, generator: G) -> &mut Self {
		self.init_resource::<TileGeneratorRegistry>();
		self.world_mut().resource_mut::<TileGeneratorRegistry>().insert(tile_class_id, generator);
		self
	}
}
