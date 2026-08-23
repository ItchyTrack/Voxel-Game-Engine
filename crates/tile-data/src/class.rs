use std::{any::Any, sync::Arc};

use bevy::prelude::*;

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

pub trait TileBuildingData: Any + Send + Sync {
	fn as_any(&self) -> &dyn Any;
	fn eq(&self, other: &dyn TileBuildingData) -> bool;
}

impl<T> TileBuildingData for T
where
	T: Any + Send + Sync + PartialEq,
{
	fn as_any(&self) -> &dyn Any {
		self
	}

	fn eq(&self, other: &dyn TileBuildingData) -> bool {
		other
			.as_any()
			.downcast_ref::<T>()
			.is_some_and(|other| self == other)
	}
}

#[derive(Component, Clone)]
pub struct TileBuildingParameters {
	data: Arc<dyn TileBuildingData>,
}

impl PartialEq for TileBuildingParameters {
	fn eq(&self, other: &Self) -> bool {
		self.data.eq(&*other.data)
	}
}

impl Eq for TileBuildingParameters {}

impl TileBuildingParameters {
	pub fn new<T: TileBuildingData>(data: T) -> Self {
		Self {
			data: Arc::new(data),
		}
	}

	pub fn downcast_ref<T: TileBuildingData>(&self) -> Option<&T> {
		self.data.as_any().downcast_ref()
	}
}
