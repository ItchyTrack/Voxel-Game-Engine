use std::sync::{Arc, RwLock};

use bevy::prelude::*;
use rustc_hash::FxHashMap;
use tile_data::{TileAppExt, TileClassId, TileGenerationSession, TileGenerator};
use voxel_data::voxels::VoxelTypeId;

#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub struct RenderingTileClass(pub TileClassId);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RenderingType {
	Ray,
	Raster,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RenderingContext {
	pub rendering_type: RenderingType,
}

#[derive(Resource, Clone, Default)]
pub struct RenderingGeneratorRegistry {
	generators: Arc<RwLock<FxHashMap<(RenderingType, VoxelTypeId), Arc<dyn TileGenerator>>>>,
}

impl RenderingGeneratorRegistry {
	pub fn insert<G: TileGenerator>(&self, rendering_type: RenderingType, source_voxel_type: VoxelTypeId, generator: G) {
		let key = (rendering_type, source_voxel_type);
		let mut generators = self.generators.write().unwrap();
		assert!(!generators.contains_key(&key), "rendering generator already registered for {key:?}");
		generators.insert(key, Arc::new(generator));
	}

	fn generator(&self, rendering_type: RenderingType, source_voxel_type: VoxelTypeId) -> Arc<dyn TileGenerator> {
		let key = (rendering_type, source_voxel_type);
		self.generators.read().unwrap().get(&key).cloned()
			.unwrap_or_else(|| panic!("no rendering generator registered for {key:?}"))
	}
}

#[derive(Clone)]
pub struct RenderingTileGenerator {
	source_voxel_type: VoxelTypeId,
	generators: RenderingGeneratorRegistry,
}

impl RenderingTileGenerator {
	pub fn new(source_voxel_type: VoxelTypeId, generators: RenderingGeneratorRegistry) -> Self {
		Self { source_voxel_type, generators }
	}
}

#[tile_data::async_trait]
impl TileGenerator for RenderingTileGenerator {
	async fn generate(&self, session: TileGenerationSession) -> Option<Box<dyn tile_data::TileData>> {
		let rendering_type = session.context::<RenderingContext>().rendering_type;
		self.generators.generator(rendering_type, self.source_voxel_type).generate(session).await
	}
}

pub trait RenderingGeneratorAppExt {
	fn register_rendering_generator<G: TileGenerator>(
		&mut self,
		rendering_type: RenderingType,
		source_voxel_type: VoxelTypeId,
		generator: G,
	) -> &mut Self;
}

impl RenderingGeneratorAppExt for App {
	fn register_rendering_generator<G: TileGenerator>(
		&mut self,
		rendering_type: RenderingType,
		source_voxel_type: VoxelTypeId,
		generator: G,
	) -> &mut Self {
		self.init_resource::<RenderingGeneratorRegistry>();
		self.world().resource::<RenderingGeneratorRegistry>().insert(rendering_type, source_voxel_type, generator);
		self
	}
}

#[derive(Default)]
pub struct RenderingGenerationPlugin;

impl Plugin for RenderingGenerationPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<RenderingGeneratorRegistry>();
		let class = app.register_tile_class();
		app.insert_resource(RenderingTileClass(class));
	}
}
