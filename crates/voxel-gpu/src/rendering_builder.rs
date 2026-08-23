use std::sync::{Arc, RwLock};

use bevy::prelude::*;
use rustc_hash::FxHashMap;
use tile_data::{TileAppExt, TileBuilder, TileBuildingSession, TileClassId};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RenderingType(usize);

#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub struct RenderingTileClass(pub TileClassId);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RenderingContext {
	pub rendering_type: RenderingType,
}

#[derive(Default)]
struct RenderingBuilders {
	next_type: usize,
	builders: FxHashMap<RenderingType, Arc<dyn TileBuilder>>,
}

#[derive(Resource, Clone, Default)]
pub struct RenderingBuilderRegistry {
	builders: Arc<RwLock<RenderingBuilders>>,
}

impl RenderingBuilderRegistry {
	pub fn register<G: TileBuilder>(&self, builder: G) -> RenderingType {
		let mut registered = self.builders.write().unwrap();
		let rendering_type = RenderingType(registered.next_type);
		registered.next_type = registered.next_type.checked_add(1).expect("rendering type space exhausted");
		registered.builders.insert(rendering_type, Arc::new(builder));
		rendering_type
	}

	fn builder(&self, rendering_type: RenderingType) -> Arc<dyn TileBuilder> {
		self.builders.read().unwrap().builders.get(&rendering_type).cloned()
			.unwrap_or_else(|| panic!("no builder registered for rendering type {rendering_type:?}"))
	}
}

#[derive(Clone)]
pub struct RenderingTileBuilder {
	builders: RenderingBuilderRegistry,
}

impl RenderingTileBuilder {
	pub fn new(builders: RenderingBuilderRegistry) -> Self {
		Self { builders }
	}
}

#[tile_data::async_trait]
impl TileBuilder for RenderingTileBuilder {
	async fn generate(&self, session: TileBuildingSession) -> Option<Box<dyn tile_data::TileData>> {
		let rendering_type = session.context::<RenderingContext>().rendering_type;
		self.builders.builder(rendering_type).generate(session).await
	}
}

pub trait RenderingBuilderAppExt {
	fn register_rendering_builder<G: TileBuilder>(&mut self, builder: G) -> RenderingType;
}

impl RenderingBuilderAppExt for App {
	fn register_rendering_builder<G: TileBuilder>(&mut self, builder: G) -> RenderingType {
		self.init_resource::<RenderingBuilderRegistry>();
		self.world().resource::<RenderingBuilderRegistry>().register(builder)
	}
}

#[derive(Default)]
pub struct RenderingGenerationPlugin;

impl Plugin for RenderingGenerationPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<RenderingBuilderRegistry>();
		let class = app.register_tile_class();
		let builders = app.world().resource::<RenderingBuilderRegistry>().clone();
		app.register_tile_builder(class, RenderingTileBuilder::new(builders));
		app.insert_resource(RenderingTileClass(class));
	}
}
