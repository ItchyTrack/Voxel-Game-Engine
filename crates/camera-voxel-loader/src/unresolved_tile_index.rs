use std::collections::HashMap;

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
	by_grid: HashMap<GridId, PerGridIndex>,
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
	pub(crate) fn insert(&mut self, key: TileKey) {
		if key.lod > MAX_SAFE_TILE_LOD {
			return;
		}
		let grid = self.by_grid.entry(key.grid).or_default();
		grid.trees[key.lod as usize].add_area(&key.min, key.size(), PRESENT);
		grid.lod_mask |= lod_bit(key.lod);
	}

	pub(crate) fn remove(&mut self, key: TileKey) {
		if key.lod > MAX_SAFE_TILE_LOD {
			return;
		}
		let Some(grid) = self.by_grid.get_mut(&key.grid) else { return };
		grid.trees[key.lod as usize].remove_area(&key.min, key.size());
		if grid.trees[key.lod as usize].is_empty() {
			grid.lod_mask &= !lod_bit(key.lod);
		}
		if grid.lod_mask == 0 {
			self.by_grid.remove(&key.grid);
		}
	}

	pub(crate) fn for_each_in_region(
		&self,
		grid: GridId,
		region: GridRegion,
		max_lod: u8,
		skip_lod: Option<u8>,
		mut f: impl FnMut(TileKey),
	) {
		let Some(grid_index) = self.by_grid.get(&grid) else { return };
		let mut lod_mask = grid_index.lod_mask & lod_mask_through(max_lod.min(MAX_SAFE_TILE_LOD));
		if let Some(skip_lod) = skip_lod {
			lod_mask &= !lod_bit(skip_lod);
		}
		while lod_mask != 0 {
			let lod = lod_mask.trailing_zeros() as u8;
			grid_index.trees[lod as usize].for_each_occupied_tile_cover(region, 1i32 << lod, |min| {
				f(TileKey { grid, lod, min });
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
	use bevy::prelude::*;
	use voxel_data::grid_tree::GridRegion;

	use super::UnresolvedTileIndex;
	use crate::types::TileKey;

	fn tile(grid: Entity, lod: u8, min: IVec3) -> TileKey {
		TileKey { grid, lod, min }
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
		index.for_each_in_region(grid, GridRegion::from_min_size(source.min, source.size()).unwrap(), 16, Some(source.lod), |key| actual.push(key));
		actual.sort_by_key(|key| (key.lod, key.min.x, key.min.y, key.min.z));

		assert_eq!(actual, vec![fine, coarse]);
	}
}
