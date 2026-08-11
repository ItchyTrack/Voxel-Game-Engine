use std::collections::{HashMap, HashSet};

use bevy::math::IVec3;
use tile_data::{TileIndex, TileIndexKey, VoxelArea};

use crate::TileKey;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct TileDependency {
	tile: TileKey,
	area: VoxelArea,
}

impl TileIndexKey for TileDependency {
	fn min(self) -> IVec3 { self.area.min }
	fn size(self) -> IVec3 { self.area.size }
}

#[derive(Debug, Default)]
pub(crate) struct TileDependencyIndex {
	index: TileIndex<TileDependency>,
	by_tile: HashMap<TileKey, Vec<VoxelArea>>,
}

impl TileDependencyIndex {
	pub(crate) fn replace(&mut self, key: TileKey, dependencies: HashSet<VoxelArea>) {
		self.remove(key);
		let dependencies: Vec<_> = dependencies.into_iter().collect();
		for area in &dependencies { self.index.insert(TileDependency { tile: key, area: *area }); }
		self.by_tile.insert(key, dependencies);
	}

	pub(crate) fn remove(&mut self, key: TileKey) {
		let Some(dependencies) = self.by_tile.remove(&key) else { return };
		for area in dependencies { self.index.remove(TileDependency { tile: key, area }); }
	}

	pub(crate) fn tiles_for_chunk(&self, chunk: IVec3) -> impl Iterator<Item = TileKey> {
		self.index.keys_covering_point(chunk).into_iter().map(|dependency| dependency.tile)
	}
}
