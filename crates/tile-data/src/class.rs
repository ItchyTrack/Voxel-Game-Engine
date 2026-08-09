use std::{
	any::Any,
	sync::{
		Arc,
		atomic::{AtomicU64, Ordering},
	},
};

use bevy::prelude::*;
use rustc_hash::FxHashMap;
use voxel_data::voxels::VoxelTypeId;

use crate::{SharedTileGenerator, TileGenerator};

static NEXT_CONTEXT_VERSION: AtomicU64 = AtomicU64::new(1);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileClassId(pub usize);

#[derive(Resource, Default)]
pub struct TileClassRegistry {
	class_count: usize,
}

impl TileClassRegistry {
	pub fn register(&mut self) -> TileClassId {
		let id = TileClassId(self.class_count);
		self.class_count += 1;
		id
	}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileGeneratorKey {
	pub class: TileClassId,
	pub source_voxel_type: VoxelTypeId,
}

#[derive(Resource, Default)]
pub struct TileGeneratorRegistry {
	generators: FxHashMap<TileGeneratorKey, SharedTileGenerator>,
}

impl TileGeneratorRegistry {
	pub fn insert<G: TileGenerator>(&mut self, class: TileClassId, source_voxel_type: VoxelTypeId, generator: G) {
		let key = TileGeneratorKey { class, source_voxel_type };
		assert!(!self.generators.contains_key(&key), "tile generator already registered for {key:?}");
		self.generators.insert(key, Arc::new(generator));
	}

	pub fn generator(&self, class: TileClassId, source_voxel_type: VoxelTypeId) -> SharedTileGenerator {
		let key = TileGeneratorKey { class, source_voxel_type };
		self.generators.get(&key).cloned().unwrap_or_else(|| panic!("no tile generator registered for {key:?}"))
	}
}

#[derive(Component, Clone)]
pub struct TileGenerationContext {
	version: u64,
	data: Arc<dyn Any + Send + Sync>,
}

impl std::fmt::Debug for TileGenerationContext {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		f.debug_struct("TileGenerationContext")
			.field("version", &self.version)
			.field("type_id", &self.data.type_id())
			.finish()
	}
}

impl TileGenerationContext {
	pub fn new<T: Any + Send + Sync>(data: T) -> Self {
		Self {
			version: NEXT_CONTEXT_VERSION.fetch_add(1, Ordering::Relaxed),
			data: Arc::new(data),
		}
	}

	pub fn version(&self) -> u64 { self.version }

	pub fn downcast_ref<T: Any + Send + Sync>(&self) -> Option<&T> {
		self.data.downcast_ref()
	}
}

pub trait TileAppExt {
	fn register_tile_class(&mut self) -> TileClassId;
	fn register_tile_generator<G: TileGenerator>(&mut self, class: TileClassId, source_voxel_type: VoxelTypeId, generator: G) -> &mut Self;
}

impl TileAppExt for App {
	fn register_tile_class(&mut self) -> TileClassId {
		if !self.world().contains_resource::<TileClassRegistry>() {
			self.init_resource::<TileClassRegistry>();
		}
		self.world_mut().resource_mut::<TileClassRegistry>().register()
	}

	fn register_tile_generator<G: TileGenerator>(&mut self, class: TileClassId, source_voxel_type: VoxelTypeId, generator: G) -> &mut Self {
		if !self.world().contains_resource::<TileGeneratorRegistry>() {
			self.init_resource::<TileGeneratorRegistry>();
		}
		self.world_mut().resource_mut::<TileGeneratorRegistry>().insert(class, source_voxel_type, generator);
		self
	}
}
