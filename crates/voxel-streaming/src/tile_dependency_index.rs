use std::collections::{HashMap, HashSet};

use bevy::math::IVec3;
use tile_data::{ChunkRegion, NonZeroChunkRegion, TileIndex, TileIndexKey};

use crate::TileKey;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct TileDependency {
	pub(crate) area: NonZeroChunkRegion,
	pub(crate) generation: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct IndexedTileDependency {
	tile: TileKey,
	dependency: TileDependency,
}

impl TileIndexKey for IndexedTileDependency {
	fn region(self) -> NonZeroChunkRegion { self.dependency.area }
}

#[derive(Debug, Default)]
pub(crate) struct TileDependencyIndex {
	index: TileIndex<IndexedTileDependency>,
	by_tile: HashMap<TileKey, Vec<TileDependency>>,
}

impl TileDependencyIndex {
	pub(crate) fn replace(&mut self, key: TileKey, dependencies: HashSet<TileDependency>) {
		self.remove(key);
		let dependencies: Vec<_> = dependencies.into_iter().collect();
		for dependency in &dependencies {
			self.index.insert(IndexedTileDependency { tile: key, dependency: *dependency });
		}
		self.by_tile.insert(key, dependencies);
	}

	pub(crate) fn remove(&mut self, key: TileKey) {
		let Some(dependencies) = self.by_tile.remove(&key) else { return };
		for dependency in dependencies {
			self.index.remove(IndexedTileDependency { tile: key, dependency });
		}
	}

	pub(crate) fn stale_tiles(&self, region: NonZeroChunkRegion, generation: u64) -> impl Iterator<Item = TileKey> {
		let mut stale = Vec::new();
		self.index.for_each_overlapping(region, |dependency| {
			if dependency.dependency.generation < generation { stale.push(dependency.tile); }
		});
		stale.into_iter()
	}
}
