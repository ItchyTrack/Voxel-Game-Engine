use std::{any::Any, fmt};

use bevy::prelude::*;
use voxel_data::grid::GridId;

use crate::TileKey;

pub trait TileData: Send + Sync + 'static {
	fn as_any(&self) -> &dyn Any;
}

#[derive(Component)]
pub struct DynamicTileData {
	data: Box<dyn TileData>,
}

impl fmt::Debug for DynamicTileData {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		f.debug_struct("DynamicTileData").field("type_id", &self.data.as_any().type_id()).finish()
	}
}

impl DynamicTileData {
	pub fn new(data: Box<dyn TileData>) -> Self { Self { data } }
	pub fn data(&self) -> &dyn TileData { self.data.as_ref() }
	pub fn downcast_ref<T: TileData>(&self) -> Option<&T> { self.data.as_any().downcast_ref() }
}

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoadedTile {
	pub grid: GridId,
	pub key: TileKey,
}
