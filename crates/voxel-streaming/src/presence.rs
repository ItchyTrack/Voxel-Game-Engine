use bevy::math::IVec3;
use bevy::transform::components::Transform;

use tile_data::NonZeroChunkRegion;
use voxel_data::region::NonZeroVoxelRegion;
use voxel_data::signed_grid_tree::SignedGridTree;
use voxel_data::grid_tree::U16Cell;

type ChunkGridTree = SignedGridTree<U16Cell>;

pub struct ChunkPresence {
	tree: ChunkGridTree,
}

impl Default for ChunkPresence {
	fn default() -> Self {
		Self { tree: ChunkGridTree::new() }
	}
}

impl ChunkPresence {
	pub fn len(&self) -> u64 {
		self.tree.len()
	}

	pub fn is_empty(&self) -> bool {
		self.tree.is_empty()
	}

	pub fn mark_present(&mut self, chunk: IVec3) {
		self.mark_present_area(NonZeroChunkRegion::from_single(chunk));
	}

	pub fn mark_present_area(&mut self, region: NonZeroChunkRegion) {
		self.tree.add_area(NonZeroVoxelRegion::new(region.min(), region.size()).unwrap(), 0u16);
	}

	pub fn clear_present(&mut self, chunk: IVec3) {
		self.clear_present_area(NonZeroChunkRegion::from_single(chunk));
	}

	pub fn clear_present_area(&mut self, region: NonZeroChunkRegion) {
		self.tree.remove_area(NonZeroVoxelRegion::new(region.min(), region.size()).unwrap());
	}

	pub fn is_present(&self, chunk: IVec3) -> bool {
		self.tree.get(chunk).is_some()
	}

	pub fn any_present_in_region(&self, region: NonZeroChunkRegion) -> bool {
		self.tree.any_in_region(NonZeroVoxelRegion::new(region.min(), region.size()).unwrap())
	}

	pub fn all_present_in_region(&self, region: NonZeroChunkRegion) -> bool {
		self.tree.is_area_filled(NonZeroVoxelRegion::new(region.min(), region.size()).unwrap())
	}

	pub fn for_each_occupied_tile_cover(&self, region: NonZeroChunkRegion, tile_size: u32, f: impl FnMut(IVec3)) {
		self.tree.for_each_occupied_tile_cover(NonZeroVoxelRegion::new(region.min(), region.size()).unwrap(), tile_size, f);
	}

	/// `transform` rotation maps +Z onto the ray direction (matches the voxel tree).
	pub fn raycast(&self, transform: &Transform, max_length: Option<f32>) -> Option<(IVec3, f32)> {
		self.tree.raycast(transform, max_length).map(|(pos, _, dist)| (pos, dist))
	}

	pub fn iter_present(&self) -> impl Iterator<Item = (IVec3, u32)> + '_ {
		self.tree.iter().map(|(origin, size, _)| (origin, size))
	}

	/// Visit every present chunk inside the inclusive chunk region `[min, max]`,
	/// descending only into subtrees that overlap it. Used for collision
	/// broad-phase so cost scales with the region, not the whole footprint.
	pub fn for_each_in_region(&self, min: IVec3, max: IVec3, mut f: impl FnMut(IVec3)) {
		let Some(region) = NonZeroVoxelRegion::from_min_max(min, max) else { return };
		self.tree.for_each_in_region(region, |origin, size, _| {
			let lo = origin.max(min);
			let hi = (origin + IVec3::splat(size as i32) - IVec3::ONE).min(max);
			for x in lo.x..=hi.x {
				for y in lo.y..=hi.y {
					for z in lo.z..=hi.z {
						f(IVec3::new(x, y, z));
					}
				}
			}
		});
	}

	/// Visit every occupied presence node or leaf whose box intersects the inclusive chunk region `[min, max]`.
	pub fn for_each_node_in_region(&self, min: IVec3, max: IVec3, f: impl FnMut(IVec3, u32, bool)) {
		let Some(region) = NonZeroVoxelRegion::from_min_max(min, max) else { return };
		self.tree.for_each_node_in_region(region, f);
	}
}
