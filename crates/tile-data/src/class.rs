use std::{any::Any, sync::Arc};

use bevy::prelude::*;
use rustc_hash::FxHashMap;
use voxel_data::voxels::VoxelTypeId;

use crate::{SharedTileGenerator, TileGenerator};

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

pub trait TileGenerationData: Any + Send + Sync {
	fn as_any(&self) -> &dyn Any;
    fn eq(&self, other: &dyn TileGenerationData) -> bool;
}

impl<T> TileGenerationData for T
where
    T: Any + Send + Sync + PartialEq,
{
	fn as_any(&self) -> &dyn Any {
        self
    }

    fn eq(&self, other: &dyn TileGenerationData) -> bool {
        other
			.as_any()
            .downcast_ref::<T>()
            .is_some_and(|other| self == other)
    }
}

#[derive(Component, Clone)]
pub struct TileGenerationParameters {
    data: Arc<dyn TileGenerationData>,
}

impl PartialEq for TileGenerationParameters {
    fn eq(&self, other: &Self) -> bool {
        self.data.eq(&*other.data)
    }
}

impl Eq for TileGenerationParameters {}

impl TileGenerationParameters {
	pub fn new<T: TileGenerationData>(data: T) -> Self {
		Self {
			data: Arc::new(data),
		}
	}

	pub fn downcast_ref<T: TileGenerationData>(&self) -> Option<&T> {
		self.data.as_any().downcast_ref()
	}
}

pub trait TileAppExt {
	fn register_tile_class(&mut self) -> TileClassId;
	fn register_tile_generator<G: TileGenerator>(&mut self, class: TileClassId, source_voxel_type: VoxelTypeId, generator: G) -> &mut Self;
}

impl TileAppExt for App {
	fn register_tile_class(&mut self) -> TileClassId {
		self.init_resource::<TileClassRegistry>();
		self.world_mut().resource_mut::<TileClassRegistry>().register()
	}

	fn register_tile_generator<G: TileGenerator>(&mut self, class: TileClassId, source_voxel_type: VoxelTypeId, generator: G) -> &mut Self {
		self.init_resource::<TileGeneratorRegistry>();
		self.world_mut().resource_mut::<TileGeneratorRegistry>().insert(class, source_voxel_type, generator);
		self
	}
}
