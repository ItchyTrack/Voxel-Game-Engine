use std::sync::{Arc, RwLock};

use bevy::prelude::*;
use rustc_hash::FxHashMap;
use tile_data::{TileAppExt, TileClassId, TileGenerationSession, TileGenerator};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RenderingType(usize);

#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub struct RenderingTileClass(pub TileClassId);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RenderingContext {
	pub rendering_type: RenderingType,
}

#[derive(Default)]
struct RenderingGenerators {
	next_type: usize,
	generators: FxHashMap<RenderingType, Arc<dyn TileGenerator>>,
}

#[derive(Resource, Clone, Default)]
pub struct RenderingGeneratorRegistry {
	generators: Arc<RwLock<RenderingGenerators>>,
}

impl RenderingGeneratorRegistry {
	pub fn register<G: TileGenerator>(&self, generator: G) -> RenderingType {
		let mut registered = self.generators.write().unwrap();
		let rendering_type = RenderingType(registered.next_type);
		registered.next_type = registered.next_type.checked_add(1).expect("rendering type space exhausted");
		registered.generators.insert(rendering_type, Arc::new(generator));
		rendering_type
	}

	fn generator(&self, rendering_type: RenderingType) -> Arc<dyn TileGenerator> {
		self.generators.read().unwrap().generators.get(&rendering_type).cloned()
			.unwrap_or_else(|| panic!("no generator registered for rendering type {rendering_type:?}"))
	}
}

#[derive(Clone)]
pub struct RenderingTileGenerator {
	generators: RenderingGeneratorRegistry,
}

impl RenderingTileGenerator {
	pub fn new(generators: RenderingGeneratorRegistry) -> Self {
		Self { generators }
	}
}

#[tile_data::async_trait]
impl TileGenerator for RenderingTileGenerator {
	async fn generate(&self, session: TileGenerationSession) -> Option<Box<dyn tile_data::TileData>> {
		let rendering_type = session.context::<RenderingContext>().rendering_type;
		self.generators.generator(rendering_type).generate(session).await
	}
}

pub trait RenderingGeneratorAppExt {
	fn register_rendering_generator<G: TileGenerator>(&mut self, generator: G) -> RenderingType;
}

impl RenderingGeneratorAppExt for App {
	fn register_rendering_generator<G: TileGenerator>(&mut self, generator: G) -> RenderingType {
		self.init_resource::<RenderingGeneratorRegistry>();
		self.world().resource::<RenderingGeneratorRegistry>().register(generator)
	}
}

#[derive(Default)]
pub struct RenderingGenerationPlugin;

impl Plugin for RenderingGenerationPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<RenderingGeneratorRegistry>();
		let class = app.register_tile_class();
		let generators = app.world().resource::<RenderingGeneratorRegistry>().clone();
		app.register_tile_generator(class, RenderingTileGenerator::new(generators));
		app.insert_resource(RenderingTileClass(class));
	}
}
