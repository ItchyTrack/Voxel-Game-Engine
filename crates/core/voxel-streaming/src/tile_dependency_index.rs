use std::collections::HashMap;

use bevy::{ecs::component::ComponentId, prelude::*};
use bile_math::NonZeroRegion;
use bile_tiles::{TileIndex, TileIndexKey};
use rustc_hash::FxHashSet;

pub(crate) type TileComponent = (Entity, ComponentId);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct TileDependency {
	tile: TileComponent,
	region: NonZeroRegion,
}

impl TileIndexKey for TileDependency {
	fn region(self) -> NonZeroRegion { self.region }
}

#[derive(Debug, Default)]
pub(crate) struct TileDependencyIndex {
	index: TileIndex<TileDependency>,
	by_tile: HashMap<TileComponent, Vec<NonZeroRegion>>,
}

impl TileDependencyIndex {
	pub(crate) fn add(&mut self, tile: TileComponent, region: NonZeroRegion) {
		self.index.insert(TileDependency { tile, region });
		self.by_tile.entry(tile).or_default().push(region);
	}

	pub(crate) fn remove(&mut self, tile: TileComponent) {
		let Some(regions) = self.by_tile.remove(&tile) else { return };
		for region in regions {
			self.index.remove(TileDependency { tile, region });
		}
	}

	pub(crate) fn tiles_using_region(&self, region: NonZeroRegion) -> FxHashSet<TileComponent> {
		let mut tiles = FxHashSet::default();
		self.index.for_each_overlapping(region, |dependency| { tiles.insert(dependency.tile); });
		tiles
	}
}
