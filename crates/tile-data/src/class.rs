use std::sync::Arc;

use bevy::prelude::*;

use crate::TileGenerator;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileClassId(pub usize);

#[derive(Resource, Default)]
pub struct TileClassRegistry {
	generators: Vec<Arc<dyn TileGenerator>>,
}

impl TileClassRegistry {
	pub fn register<G: TileGenerator>(&mut self, generator: G) -> TileClassId {
		let id = TileClassId(self.generators.len());
		self.generators.push(Arc::new(generator));
		id
	}

	pub fn generator(&self, class: TileClassId) -> Option<Arc<dyn TileGenerator>> {
		self.generators.get(class.0).cloned()
	}
}

pub trait TileAppExt {
	fn register_tile_class<G: TileGenerator>(&mut self, generator: G) -> TileClassId;
}

impl TileAppExt for App {
	fn register_tile_class<G: TileGenerator>(&mut self, generator: G) -> TileClassId {
		if !self.world().contains_resource::<TileClassRegistry>() {
			self.init_resource::<TileClassRegistry>();
		}
		self.world_mut().resource_mut::<TileClassRegistry>().register(generator)
	}
}
