use super::*;

impl<C: GridCell, Co: GridCoord> GridTree<C, Co> {
	pub fn fill_region(&mut self, region: GridRegion, data: C::Data) {
		self.add_regions(&[(region, data)]);
	}

	pub fn add_area(&mut self, pos: &Co::Pos, size: IVec3, data: C::Data) {
		let Some(region) = GridRegion::from_min_size(Co::to_ivec3(*pos), size) else { return };
		self.fill_region(region, data);
	}

	pub fn add_regions(&mut self, regions: &[(GridRegion, C::Data)]) {
		let mut ops = Vec::with_capacity(regions.len());
		let mut bounds: Option<GridRegion> = None;
		for (region, data) in regions {
			debug_assert!(*data <= C::MAX_DATA);
			ops.push(AreaOp { min: region.min, end: region.end, data: *data });
			bounds = Some(match bounds {
				Some(bounds) => GridRegion { min: bounds.min.min(region.min), end: bounds.end.max(region.end) },
				None => *region,
			});
		}
		let Some(bounds) = bounds else { return };
		self.apply_area_ops(&ops, bounds);
	}

	pub fn add_single_voxels(&mut self, voxels: &[(Co::Pos, C::Data)]) {
		let mut bounds: Option<(IVec3, IVec3)> = None;
		for (pos, data) in voxels {
			debug_assert!(*data <= C::MAX_DATA);
			let pos = Co::to_ivec3(*pos);
			bounds = Some(match bounds {
				Some((lo, hi)) => (lo.min(pos), hi.max(pos)),
				None => (pos, pos),
			});
		}
		let Some((min, max)) = bounds else { return };
		self.add_single_voxels_in_bounds(voxels, min, max);
	}

	pub fn add_single_voxels_in_bounds(&mut self, voxels: &[(Co::Pos, C::Data)], min: IVec3, max: IVec3) {
		self.include_bounding_box(min, max);
		for (_, data) in voxels {
			debug_assert!(*data <= C::MAX_DATA);
		}
		if self.is_empty() && self.try_build_single_voxel_pairs(min, max, voxels) {
			return;
		}
		if !self.make_sure_root_covers_area(min, max) {
			return;
		}
		for (pos, data) in voxels {
			self.insert(pos, *data);
		}
	}

	pub fn add_areas(&mut self, areas: &[(Co::Pos, IVec3, C::Data)]) {
		let mut regions = Vec::with_capacity(areas.len());
		for (pos, size, data) in areas {
			let Some(region) = GridRegion::from_min_size(Co::to_ivec3(*pos), *size) else { continue };
			regions.push((region, *data));
		}
		self.add_regions(&regions);
	}

	pub(super) fn apply_area_ops(&mut self, ops: &[AreaOp<C::Data>], bounds: GridRegion) {
		if ops.is_empty() {
			return;
		}
		self.include_bounding_box(bounds.min, bounds.max_inclusive());
		if self.is_empty() && ops.iter().all(|op| op.end - op.min == IVec3::ONE) && self.try_build_single_voxel_batch(bounds.min, bounds.max_inclusive(), ops) {
			return;
		}
		if !self.make_sure_root_covers_area(bounds.min, bounds.max_inclusive()) || !self.has_node_budget() {
			return;
		}

		for attempt in 0..3 {
			if self.add_areas_recurse(0, self.root_depth, self.root_pos, ops) {
				return;
			}
			if attempt < 2 {
				self.compact();
			}
		}
		bevy::log::warn!("GridTree could not finish add_areas after compaction retries");
	}
}
