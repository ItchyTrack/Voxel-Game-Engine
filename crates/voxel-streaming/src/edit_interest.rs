use tile_data::NonZeroChunkRegion;
use voxel_data::{
	grid_tree::U32Cell,
	region::NonZeroVoxelRegion,
	signed_grid_tree::SignedGridTree,
};

#[derive(Default)]
pub struct ChunkEditInterest {
	counts: SignedGridTree<U32Cell>,
}

impl ChunkEditInterest {
	pub fn retain(&mut self, region: NonZeroChunkRegion) {
		let region = tree_region(region);
		let mut overlaps = Vec::new();
		self.counts.for_each_in_region(region, |origin, size, count| {
			let occupied = NonZeroVoxelRegion::from_min_size(origin, bevy::math::UVec3::splat(size)).unwrap();
			let overlap = occupied.intersection(region).expect("visited edit-interest cell did not overlap query");
			overlaps.push((overlap, count.checked_add(1).expect("chunk edit-interest count overflow")));
		});

		self.counts.add_area(region, 1);
		for (overlap, count) in overlaps {
			self.counts.add_area(overlap, count);
		}
	}

	pub fn release(&mut self, region: NonZeroChunkRegion) -> bool {
		let region = tree_region(region);
		if !self.counts.is_area_filled(region) { return false; }

		let mut remaining = Vec::new();
		self.counts.for_each_in_region(region, |origin, size, count| {
			if count == 1 { return; }
			let occupied = NonZeroVoxelRegion::from_min_size(origin, bevy::math::UVec3::splat(size)).unwrap();
			let overlap = occupied.intersection(region).expect("visited edit-interest cell did not overlap query");
			remaining.push((overlap, count - 1));
		});

		self.counts.remove_area(region);
		for (overlap, count) in remaining {
			self.counts.add_area(overlap, count);
		}
		true
	}

	pub fn overlaps(&self, region: NonZeroChunkRegion) -> bool {
		self.counts.any_in_region(tree_region(region))
	}
}

fn tree_region(region: NonZeroChunkRegion) -> NonZeroVoxelRegion {
	NonZeroVoxelRegion::new(region.min(), region.size()).unwrap()
}

#[cfg(test)]
mod tests {
	use bevy::math::{IVec3, UVec3};

	use super::*;

	fn region(min: IVec3, size: UVec3) -> NonZeroChunkRegion {
		NonZeroChunkRegion::from_min_size(min, size).unwrap()
	}

	#[test]
	fn releasing_one_overlapping_region_preserves_the_other() {
		let first = region(IVec3::ZERO, UVec3::splat(4));
		let second = region(IVec3::splat(2), UVec3::splat(4));
		let first_only = NonZeroChunkRegion::from_single(IVec3::ZERO);
		let overlap = NonZeroChunkRegion::from_single(IVec3::splat(3));
		let second_only = NonZeroChunkRegion::from_single(IVec3::splat(5));
		let mut interest = ChunkEditInterest::default();

		interest.retain(first);
		interest.retain(second);
		assert!(interest.release(first));

		assert!(!interest.overlaps(first_only));
		assert!(interest.overlaps(overlap));
		assert!(interest.overlaps(second_only));
	}

	#[test]
	fn duplicate_region_references_are_counted() {
		let area = region(IVec3::splat(-2), UVec3::splat(4));
		let mut interest = ChunkEditInterest::default();

		interest.retain(area);
		interest.retain(area);
		assert!(interest.release(area));
		assert!(interest.overlaps(area));
		assert!(interest.release(area));
		assert!(!interest.overlaps(area));
		assert!(!interest.release(area));
	}
}
