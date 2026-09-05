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

#[derive(Default, Component, Clone)]
pub struct TileBuildingParameters {
	data: Option<Arc<dyn TileBuildingData>>,
}

impl PartialEq for TileBuildingParameters {
	fn eq(&self, other: &Self) -> bool {
		let Some(self_data) = &self.data else { return other.data.is_none(); };
		let Some(other_data) = &other.data else { return false; };
		self_data.eq(&**other_data)
	}
}

impl Eq for TileBuildingParameters {}

impl TileBuildingParameters {
	pub fn new<T: TileBuildingData>(data: T) -> Self {
		Self {
			data: Some(Arc::new(data)),
		}
	}

	pub fn downcast_ref<T: TileBuildingData>(&self) -> Option<&T> {
		self.data.as_ref()?.as_any().downcast_ref()
	}
}
