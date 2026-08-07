use std::collections::{HashMap, HashSet};

use voxel_streaming::TileClassId;
use voxel_data::{
	grid::GridId,
	grid_tree::GridRegion,
	signed_grid_tree::SignedGridTree,
	voxel_grid_tree::PackedCell,
};

use crate::types::TileKey;

const PRESENT: u16 = 1;
const MAX_LOD_BITS: usize = u32::BITS as usize;
const MAX_SAFE_TILE_LOD: u8 = i32::BITS as u8 - 2;

#[derive(Debug, Clone, Default)]
pub(crate) struct UnresolvedTileIndex {
	by_grid: HashMap<(GridId, TileClassId), PerGridIndex>,
}

#[derive(Debug, Clone)]
struct PerGridIndex {
	lod_mask: u32,
	trees: [SignedGridTree<PackedCell>; MAX_LOD_BITS],
}

impl Default for PerGridIndex {
	fn default() -> Self {
		Self { lod_mask: 0, trees: std::array::from_fn(|_| SignedGridTree::new()) }
	}
}

impl UnresolvedTileIndex {
	pub(crate) fn contains(&self, key: TileKey) -> bool {
		key.lod <= MAX_SAFE_TILE_LOD
			&& self.by_grid.get(&(key.grid, key.class)).is_some_and(|grid| grid.trees[key.lod as usize].get(&key.min).is_some())
	}

	pub(crate) fn insert(&mut self, key: TileKey) {
		if key.lod > MAX_SAFE_TILE_LOD {
			return;
		}
		let grid = self.by_grid.entry((key.grid, key.class)).or_default();
		grid.trees[key.lod as usize].add_area(&key.min, key.size(), PRESENT);
		grid.lod_mask |= lod_bit(key.lod);
	}

	pub(crate) fn remove(&mut self, key: TileKey) -> bool {
		if !self.contains(key) {
			return false;
		}
		let Some(grid) = self.by_grid.get_mut(&(key.grid, key.class)) else { return false };
		grid.trees[key.lod as usize].remove_area(&key.min, key.size());
		if grid.trees[key.lod as usize].is_empty() {
			grid.lod_mask &= !lod_bit(key.lod);
		}
		if grid.lod_mask == 0 {
			self.by_grid.remove(&(key.grid, key.class));
		}
		true
	}

	pub(crate) fn keys(&self) -> Vec<TileKey> {
		let mut keys = HashSet::new();
		for (&(grid, class), index) in &self.by_grid {
			for lod in 0..=MAX_SAFE_TILE_LOD {
				if index.lod_mask & lod_bit(lod) == 0 { continue; }
				let tile_size = 1i32 << lod;
				for (origin, region_size, _) in index.trees[lod as usize].iter() {
					let region_size = region_size as i32;
					let first = origin.div_euclid(bevy::math::IVec3::splat(tile_size)) * tile_size;
					let end = origin + bevy::math::IVec3::splat(region_size);
					for x in (first.x..end.x).step_by(tile_size as usize) {
						for y in (first.y..end.y).step_by(tile_size as usize) {
							for z in (first.z..end.z).step_by(tile_size as usize) {
								keys.insert(TileKey { grid, class, lod, min: bevy::math::IVec3::new(x, y, z) });
							}
						}
					}
				}
			}
		}
		keys.into_iter().collect()
	}

	pub(crate) fn for_each_in_region(
		&self,
		grid: GridId,
		class: TileClassId,
		region: GridRegion,
		max_lod: u8,
		skip_lod: Option<u8>,
		mut f: impl FnMut(TileKey),
	) {
		let Some(grid_index) = self.by_grid.get(&(grid, class)) else { return };
		let mut lod_mask = grid_index.lod_mask & lod_mask_through(max_lod.min(MAX_SAFE_TILE_LOD));
		if let Some(skip_lod) = skip_lod {
			lod_mask &= !lod_bit(skip_lod);
		}
		while lod_mask != 0 {
			let lod = lod_mask.trailing_zeros() as u8;
			grid_index.trees[lod as usize].for_each_occupied_tile_cover(region, 1i32 << lod, |min| {
				f(TileKey { grid, class, lod, min });
			});
			lod_mask &= lod_mask - 1;
		}
	}
}

#[inline]
fn lod_bit(lod: u8) -> u32 {
	1u32.checked_shl(lod as u32).unwrap_or(0)
}

#[inline]
fn lod_mask_through(max_lod: u8) -> u32 {
	if max_lod as u32 >= u32::BITS - 1 {
		u32::MAX
	} else {
		(1u32 << (max_lod + 1)) - 1
	}
}

#[cfg(test)]
mod tests {
	use std::collections::HashSet;

	use bevy::prelude::*;
	use voxel_data::grid_tree::GridRegion;

	use super::UnresolvedTileIndex;
	use crate::types::TileKey;

	fn tile(grid: Entity, lod: u8, min: IVec3) -> TileKey {
		TileKey { grid, class: voxel_streaming::TileClassId(0), lod, min }
	}

	#[test]
	fn keys_reconstructs_indexed_tiles() {
		let grid = Entity::from_bits(1);
		let expected = HashSet::from([
			tile(grid, 1, IVec3::ZERO),
			tile(grid, 1, IVec3::new(2, 0, 0)),
			tile(grid, 0, IVec3::new(-1, 0, 0)),
		]);
		let mut index = UnresolvedTileIndex::default();
		for key in &expected { index.insert(*key); }

		assert_eq!(index.keys().into_iter().collect::<HashSet<_>>(), expected);
	}

	#[test]
	fn query_does_not_report_an_empty_tile_next_to_pending_coverage() {
		let grid = Entity::from_bits(1);
		let mut index = UnresolvedTileIndex::default();
		index.insert(tile(grid, 1, IVec3::new(2, 0, 0)));
		let mut actual = Vec::new();

		index.for_each_in_region(
			grid,
			voxel_streaming::TileClassId(0),
			GridRegion::from_min_size(IVec3::ZERO, IVec3::ONE).unwrap(),
			1,
			Some(0),
			|key| actual.push(key),
		);

		assert!(actual.is_empty(), "query returned phantom replacement tiles: {actual:?}");
	}

	#[test]
	fn query_returns_only_actual_overlapping_tiles() {
		let grid = Entity::from_bits(1);
		let source = tile(grid, 3, IVec3::ZERO);
		let fine = tile(grid, 0, IVec3::new(2, 2, 2));
		let coarse = tile(grid, 4, IVec3::ZERO);
		let disjoint = tile(grid, 0, IVec3::new(20, 20, 20));
		let mut index = UnresolvedTileIndex::default();
		index.insert(fine);
		index.insert(coarse);
		index.insert(disjoint);

		let mut actual = Vec::new();
		index.for_each_in_region(grid, source.class, GridRegion::from_min_size(source.min, source.size()).unwrap(), 16, Some(source.lod), |key| actual.push(key));
		actual.sort_by_key(|key| (key.lod, key.min.x, key.min.y, key.min.z));

		assert_eq!(actual, vec![fine, coarse]);
	}
}
