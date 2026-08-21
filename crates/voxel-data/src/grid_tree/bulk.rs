use bevy::math::UVec3;

use super::*;

impl<G: GridType, Co: GridCoord> GridTree<G, Co> {
	pub fn fill_region(&mut self, region: NonZeroVoxelRegion, data: G::Data<'_>) {
		self.add_regions(&[(region, data)]);
	}

	pub fn add_area(&mut self, pos: &Co::Pos, size: UVec3, data: G::Data<'_>) {
		let Some(region) = NonZeroVoxelRegion::from_min_size(Co::to_ivec3(*pos), size) else { return };
		self.fill_region(region, data);
	}

	pub fn add_regions<'a>(&mut self, regions: &[(NonZeroVoxelRegion, G::Data<'a>)]) {
		let mut ops = Vec::with_capacity(regions.len());
		let mut bounds: Option<NonZeroVoxelRegion> = None;
		for (region, data) in regions {
			ops.push(AreaOp { region: *region, data: *data });
			bounds = Some(match bounds {
				Some(bounds) => NonZeroVoxelRegion::from_min_end(bounds.min().min(region.min()), bounds.end().max(region.end())).unwrap(),
				None => *region,
			});
		}
		let Some(bounds) = bounds else { return };
		self.apply_area_ops(&ops, bounds);
	}

	pub fn add_single_voxels<'a>(&mut self, voxels: &[(Co::Pos, G::Data<'a>)]) {
		let mut bounds: Option<(IVec3, IVec3)> = None;
		for (pos, _) in voxels {
			let pos = Co::to_ivec3(*pos);
			bounds = Some(match bounds {
				Some((lo, hi)) => (lo.min(pos), hi.max(pos)),
				None => (pos, pos),
			});
		}
		let Some((min, max)) = bounds else { return };
		self.add_single_voxels_in_bounds(voxels, min, max);
	}

	pub fn add_single_voxels_in_bounds<'a>(&mut self, voxels: &[(Co::Pos, G::Data<'a>)], min: IVec3, max: IVec3) {
		if self.is_empty() && self.build_single_voxel_pairs(min, max, voxels) {
			return;
		}
		if !self.make_sure_root_covers_area(min, max) {
			return;
		}
		for (pos, data) in voxels {
			self.insert(pos, *data);
		}
	}

	pub fn add_areas<'a>(&mut self, areas: &[(Co::Pos, UVec3, G::Data<'a>)]) {
		let mut regions = Vec::with_capacity(areas.len());
		for (pos, size, data) in areas {
			let Some(region) = NonZeroVoxelRegion::from_min_size(Co::to_ivec3(*pos), *size) else { continue };
			regions.push((region, *data));
		}
		self.add_regions(&regions);
	}

	pub(super) fn apply_area_ops<'a>(&mut self, ops: &[AreaOp<'a, G>], bounds: NonZeroVoxelRegion) {
		if ops.is_empty() {
			return;
		}
		if self.is_empty() && ops.iter().all(|op| op.region.end() - op.region.min() == IVec3::ONE) && self.build_single_voxel_batch(bounds.min(), bounds.max(), ops) {
			return;
		}
		if !self.make_sure_root_covers_area(bounds.min(), bounds.max()) || !self.has_node_budget() {
			return;
		}

		let _ = self.add_areas_recurse(0, self.raw.root_depth(), self.raw.root_pos(), ops);
	}
}
