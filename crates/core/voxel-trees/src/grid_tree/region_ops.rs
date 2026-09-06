use bevy::math::{IVec2, IVec3, UVec3, Vec3};

use crate::sdf::{shrink_aabb_with_sdf, voxel_center, voxel_region_from_bounds, Sdf};

use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BlockSdfRelation {
	Outside,
	Inside,
	Intersecting,
}

fn sdf_relation_for_block(sdf: &(impl Sdf + ?Sized), block_min: UVec3, block_size: u32) -> BlockSdfRelation {
	let min = block_min.as_vec3();
	let size = Vec3::splat(block_size as f32);
	let center = min + size * 0.5;
	let radius = size.length() * 0.5;
	let d = sdf.sample(center);
	if d > radius {
		BlockSdfRelation::Outside
	} else if d < -radius {
		BlockSdfRelation::Inside
	} else {
		BlockSdfRelation::Intersecting
	}
}

impl<G: GridType> GridTree64<G> {
	pub fn apply_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize, data: G::Data<'_>) {
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, sdf, face_resolution, iterations);
		let Some(region) = voxel_region_from_bounds(min, max) else { return };
		self.fill_sdf_region(region, sdf, data);
	}

	pub fn clear_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize) {
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, sdf, face_resolution, iterations);
		let Some(region) = voxel_region_from_bounds(min, max) else { return };
		self.clear_sdf_region(region, sdf);
	}

	fn fill_sdf_region(&mut self, region: NonZeroVoxelRegion, sdf: &(impl Sdf + ?Sized), data: G::Data<'_>) {
		assert!(region.min().is_negative_bitmask() == 0);
		if !self.make_sure_root_covers_area(region.min().as_uvec3(), region.max().as_uvec3()) || !self.has_node_budget() {
			return;
		}

		let _ = self.fill_sdf_recurse(0, self.raw.root_depth(), self.raw.root_pos(), region, sdf, data);
	}

	fn clear_sdf_region(&mut self, region: NonZeroVoxelRegion, sdf: &(impl Sdf + ?Sized)) {
		assert!(region.min().is_negative_bitmask() == 0);
		if self.is_empty() {
			return;
		}
		let Some(region) = self.root_region().intersection(region) else { return };
		if !self.has_node_budget() {
			return;
		}

		let _ = self.clear_sdf_recurse(0, self.raw.root_depth(), self.raw.root_pos(), region, sdf);
	}

	fn fill_sdf_recurse(
		&mut self,
		node_index: u32,
		node_depth: u8,
		node_origin: UVec3,
		region: NonZeroVoxelRegion,
		sdf: &(impl Sdf + ?Sized),
		data: G::Data<'_>,
	) -> bool {
		assert!(region.min().is_negative_bitmask() == 0);
		let node_region = NonZeroVoxelRegion::from_min_size(node_origin.as_ivec3(), UVec3::splat(size(node_depth))).unwrap();
		let Some(overlap) = node_region.intersection(region) else { return true };
		if region.contains_region(node_region) {
			match sdf_relation_for_block(sdf, node_origin, size(node_depth)) {
				BlockSdfRelation::Outside => return true,
				BlockSdfRelation::Inside => {
					if node_index == 0 {
						for z in 0..SIZE {
							for y in 0..SIZE {
								for x in 0..SIZE {
									let child_index = (x + y * SIZE + z * SIZE * SIZE) as u8;
									self.set_child_area_to_data(node_index, node_depth, child_index, data);
								}
							}
						}
						return true;
					}
					return false;
				}
				BlockSdfRelation::Intersecting => {}
			}
		}
		let cell_size = child_size(node_depth);
		let child_min = (overlap.min().as_uvec3() - node_origin) / UVec3::splat(cell_size);
		let child_max = (overlap.end().as_uvec3() - node_origin - UVec3::ONE) / UVec3::splat(cell_size);

		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as u8;
					let child_origin = node_origin + UVec3::new(x, y, z) * cell_size;
					let child_region = NonZeroVoxelRegion::from_min_size(child_origin.as_ivec3(), UVec3::splat(cell_size)).unwrap();
					match sdf_relation_for_block(sdf, child_origin, cell_size) {
						BlockSdfRelation::Outside => continue,
						BlockSdfRelation::Inside if region.contains_region(child_region) => {
							self.set_child_area_to_data(node_index, node_depth, child_index, data);
							continue;
						}
						BlockSdfRelation::Inside | BlockSdfRelation::Intersecting => {}
					}
					if node_depth == 0 {
						if sdf.sample(voxel_center(child_origin.as_ivec3())) <= 0.0 {
							self.set_voxel_child_to_data(node_index, child_index, data);
						}
						continue;
					}

					let child_node_index = match self.child_node_for_partial_area(node_index, child_index) {
						Some(index) => index,
						None => return false,
					};
					if !self.fill_sdf_recurse(child_node_index, node_depth - 1, child_origin, region, sdf, data) {
						return false;
					}
					self.collapse_child_node_if_possible(node_index, child_index);
				}
			}
		}
		true
	}

	fn clear_sdf_recurse(
		&mut self,
		node_index: u32,
		node_depth: u8,
		node_origin: UVec3,
		region: NonZeroVoxelRegion,
		sdf: &(impl Sdf + ?Sized),
	) -> bool {
		let node_region = NonZeroVoxelRegion::from_min_size(node_origin.as_ivec3(), UVec3::splat(size(node_depth))).unwrap();
		let Some(overlap) = node_region.intersection(region) else { return true };
		if region.contains_region(node_region) {
			match sdf_relation_for_block(sdf, node_origin, size(node_depth)) {
				BlockSdfRelation::Outside => return true,
				BlockSdfRelation::Inside => {
					if node_index == 0 {
						for z in 0..SIZE as i32 {
							for y in 0..SIZE as i32 {
								for x in 0..SIZE as i32 {
									let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
									self.set_child_area_to_empty(node_index, node_depth, child_index);
								}
							}
						}
						return true;
					}
					return false;
				}
				BlockSdfRelation::Intersecting => {}
			}
		}
		let cell_size = child_size(node_depth);
		let child_min = (overlap.min().as_uvec3() - node_origin) / UVec3::splat(cell_size);
		let child_max = (overlap.end().as_uvec3() - node_origin - UVec3::ONE) / UVec3::splat(cell_size);

		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as u8;
					let child_origin = node_origin + UVec3::new(x, y, z) * cell_size;
					let child_region = NonZeroVoxelRegion::from_min_size(child_origin.as_ivec3(), UVec3::splat(cell_size)).unwrap();
					let cell_kind = self.raw.cell_kind(node_index, child_index);
					if cell_kind == CellKind::Empty {
						continue;
					}
					match sdf_relation_for_block(sdf, child_origin, cell_size) {
						BlockSdfRelation::Outside => continue,
						BlockSdfRelation::Inside if region.contains_region(child_region) => {
							self.set_child_area_to_empty(node_index, node_depth, child_index);
							continue;
						}
						BlockSdfRelation::Inside | BlockSdfRelation::Intersecting => {}
					}
					if node_depth == 0 {
						if sdf.sample(voxel_center(child_origin.as_ivec3())) <= 0.0 {
							self.set_child_area_to_empty(node_index, node_depth, child_index);
						}
						continue;
					}

					if cell_kind != CellKind::Node && !region.intersects(child_region) {
						continue;
					}

					let child_node_index = match cell_kind {
						CellKind::Node => self.raw.child_index(node_index, child_index),
						_ => match self.child_node_for_partial_area(node_index, child_index) {
							Some(index) => index,
							None => return false,
						},
					};
					if !self.clear_sdf_recurse(child_node_index, node_depth - 1, child_origin, region, sdf) {
						return false;
					}
					self.collapse_child_node_if_possible(node_index, child_index);
				}
			}
		}
		true
	}

	pub fn split_region(&mut self, region: NonZeroVoxelRegion) -> Self {
		assert!(region.min().is_negative_bitmask() == 0);
		let mut out = Self::new_with_type(self.grid_type.clone());
		if self.is_empty() {
			return out;
		}
		let Some(region) = self.root_region().intersection(region) else { return out };
		out.merge_region_from(self, region, IVec3::ZERO);
		self.clear_region(region);
		out
	}

	pub fn merge_tree(&mut self, other: &Self, offset: IVec3) {
		self.merge_region_from(other, other.root_region(), offset);
	}

	pub fn merge_region_from(&mut self, other: &Self, source_region: NonZeroVoxelRegion, offset: IVec3) {
		assert!(source_region.min().is_negative_bitmask() == 0);
		if other.is_empty() {
			return;
		}
		let Some(source_region) = source_region.intersection(other.root_region()) else { return };
		if self.is_empty() && source_region.contains_region(other.root_region()) {
			*self = other.clone();
			self.raw.set_root((other.raw.root_pos().as_ivec3() + offset).as_uvec3(), other.raw.root_depth());
			return;
		}
		let Some(source_bounds) = other.occupied_bounds_in_region(source_region) else { return };
		self.merge_region_from_with_bounds(other, source_region, source_bounds, offset, false);
	}

	pub fn overwrite_region_from(&mut self, other: &Self, source_region: NonZeroVoxelRegion, offset: IVec3) {
		assert!(source_region.min().is_negative_bitmask() == 0);
		let destination_region = source_region.translated(offset);
		let source_bounds = source_region
			.intersection(other.root_region())
			.filter(|_| !other.is_empty())
			.and_then(|region| other.occupied_bounds_in_region(region));
		let Some(source_bounds) = source_bounds else {
			self.clear_region(destination_region);
			return;
		};
		if self.occupied_bounds().is_none_or(|bounds| destination_region.contains_region(bounds)) {
			let grid_type = self.grid_type.clone();
			*self = Self::new_with_type(grid_type);
			self.merge_region_from(other, source_bounds, offset);
			return;
		}
		self.clear_region_outside(destination_region, source_bounds.translated(offset));
		self.merge_region_from_with_bounds(other, source_bounds, source_bounds, offset, true);
	}

	pub fn merge_region_from_mapped<'a>(&'a mut self, other: &'a Self, source_region: NonZeroVoxelRegion, offset: IVec3, mut map: impl FnMut(G::Data<'a>) -> G::Data<'a>) {
		assert!(source_region.min().is_negative_bitmask() == 0);
		if other.is_empty() { return; }
		let Some(source_region) = source_region.intersection(other.root_region()) else { return };
		let Some(source_bounds) = other.occupied_bounds_in_region(source_region) else { return };
		let dest_bounds = source_bounds.translated(offset);
		if !self.make_sure_root_covers_area(dest_bounds.min().as_uvec3(), dest_bounds.max().as_uvec3()) { return; }
		let leaves: Vec<_> = other.leaves().filter_map(|leaf| {
			let leaf_region = NonZeroVoxelRegion::from_min_size(leaf.origin.as_ivec3(), UVec3::splat(leaf.size())).unwrap();
			leaf_region.intersection(source_region).map(|clipped| (clipped, other.cell_data(leaf.child_handle.unwrap(), leaf.child_index)))
		}).collect();
		for (clipped, data) in leaves {
			self.fill_region(clipped.translated(offset), map(data));
		}
	}

	fn merge_region_from_with_bounds(&mut self, other: &Self, source_region: NonZeroVoxelRegion, source_bounds: NonZeroVoxelRegion, offset: IVec3, overwrite: bool) {
		assert!(source_region.min().is_negative_bitmask() == 0);
		assert!(source_bounds.min().is_negative_bitmask() == 0);
		if !overwrite {
			let source_count = other.occupied_count_in_region(source_bounds);
			let walk_destination = !self.is_empty() && self.len() < source_count;
			if walk_destination {
				self.clear_destination_covered_by_source(other, source_bounds, offset);
			}
		}
		self.merge_region_from_source_walk(other, source_bounds, source_region.contains_region(other.root_region()), offset, overwrite);
	}

	fn merge_region_from_source_walk(&mut self, other: &Self, source_bounds: NonZeroVoxelRegion, full_root_covered: bool, offset: IVec3, overwrite: bool) {
		assert!(source_bounds.min().is_negative_bitmask() == 0);
		let dest_bounds = source_bounds.translated(offset);
		if !self.make_sure_root_covers_area(dest_bounds.min().as_uvec3(), dest_bounds.max().as_uvec3()) || !self.has_node_budget() {
			return;
		}

		if full_root_covered {
			if let Some(dest_node_index) = self.node_for_region((other.raw.root_pos().as_ivec3() + offset).as_uvec3(), other.raw.root_depth()) {
				if self.merge_aligned_nodes_from(other, 0, dest_node_index, other.raw.root_depth(), overwrite) {
					return;
				}
			}
		}
		let _ = self.merge_region_from_recurse(other, 0, other.raw.root_depth(), other.raw.root_pos(), source_bounds, offset, overwrite);
	}

	fn node_for_region(&mut self, target_origin: UVec3, target_depth: u8) -> Option<u32> {
		let mut node_index = 0;
		let mut node_depth = self.raw.root_depth();
		let mut node_origin = self.raw.root_pos();
		while node_depth > target_depth {
			let cell_size = child_size(node_depth);
			if target_origin.cmplt(node_origin).any() { return None;}
			let rel = target_origin - node_origin;
			let child_pos = rel / UVec3::splat(cell_size);
			if child_pos.cmpge(UVec3::splat(SIZE as u32)).any() {
				return None;
			}
			let child_index = (child_pos.x + child_pos.y * SIZE as u32 + child_pos.z * SIZE as u32 * SIZE as u32) as u8;
			node_origin += child_pos * cell_size;
			node_index = self.child_node_for_partial_area(node_index, child_index)?;
			node_depth -= 1;
		}
		(node_origin == target_origin && node_depth == target_depth).then_some(node_index)
	}

	fn merge_aligned_nodes_from(&mut self, other: &Self, src_node_index: u32, dest_node_index: u32, node_depth: u8, overwrite: bool) -> bool {
		for child_index in 0..SIZE_CUBED {
			match other.raw.cell_kind(src_node_index, child_index) {
				CellKind::Empty => {
					if overwrite {
						self.set_child_area_to_empty(dest_node_index, node_depth, child_index);
					}
				}
				CellKind::Data => {
					let data = other.cell_data(src_node_index, child_index);
					self.set_child_area_to_data(dest_node_index, node_depth, child_index, data);
				}
				CellKind::Node => {
					if node_depth == 0 {
						continue;
					}
					let dest_child_node = match self.child_node_for_partial_area(dest_node_index, child_index) {
						Some(index) => index,
						None => return false,
					};
					if !self.merge_aligned_nodes_from(other, other.raw.child_index(src_node_index, child_index), dest_child_node, node_depth - 1, overwrite) {
						return false;
					}
					self.collapse_child_node_if_possible(dest_node_index, child_index);
				}
			}
		}
		true
	}

	fn merge_region_from_recurse(&mut self, other: &Self, src_node_index: u32, src_node_depth: u8, src_node_origin: UVec3, source_region: NonZeroVoxelRegion, offset: IVec3, overwrite: bool) -> bool {
		assert!(source_region.min().is_negative_bitmask() == 0);
		let node_region = NonZeroVoxelRegion::from_min_size(src_node_origin.as_ivec3(), UVec3::splat(size(src_node_depth))).unwrap();
		let Some(overlap) = source_region.intersection(node_region) else { return true };
		let cell_size = child_size(src_node_depth);
		let child_max = (overlap.end().as_uvec3() - src_node_origin - UVec3::ONE) / UVec3::splat(cell_size);
		let child_min = (overlap.min().as_uvec3() - src_node_origin) / UVec3::splat(cell_size);

		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as u8;
					let cell_kind = other.raw.cell_kind(src_node_index, child_index);
					if cell_kind == CellKind::Empty && !overwrite { continue; }
					let child_origin = src_node_origin + UVec3::new(x, y, z) * cell_size;
					let child_region = NonZeroVoxelRegion::from_min_size(child_origin.as_ivec3(), UVec3::splat(cell_size)).unwrap();
					let Some(clipped_source) = source_region.intersection(child_region) else { continue };
					match cell_kind {
						CellKind::Empty => self.clear_region(clipped_source.translated(offset)),
						CellKind::Data => {
							let data = other.cell_data(src_node_index, child_index);
							if !self.fill_region_recurse_entry(clipped_source.translated(offset), data) { return false; }
						}
						CellKind::Node => {
							let child_node_index = other.raw.child_index(src_node_index, child_index);
							let child_depth = src_node_depth - 1;
							if source_region.contains_region(child_region) {
								if let Some(dest_node_index) = self.node_for_region((child_origin.as_ivec3() + offset).as_uvec3(), child_depth) {
									if !self.merge_aligned_nodes_from(other, child_node_index, dest_node_index, child_depth, overwrite) { return false; }
									continue;
								}
							}
							if !self.merge_region_from_recurse(other, child_node_index, child_depth, child_origin, source_region, offset, overwrite) { return false; }
						}
					}
				}
			}
		}
		true
	}

	fn fill_region_recurse_entry(&mut self, region: NonZeroVoxelRegion, data: G::Data<'_>) -> bool {
		self.fill_region_recurse(0, self.raw.root_depth(), self.raw.root_pos(), region, data)
	}

	fn fill_region_recurse(&mut self, node_index: u32, node_depth: u8, node_origin: UVec3, region: NonZeroVoxelRegion, data: G::Data<'_>) -> bool {
		let node_region = NonZeroVoxelRegion::from_min_size(node_origin.as_ivec3(), UVec3::splat(size(node_depth))).unwrap();
		let Some(overlap) = region.intersection(node_region) else { return true };
		let cell_size = child_size(node_depth);
		let child_min = (overlap.min().as_uvec3() - node_origin) / UVec3::splat(cell_size);
		let child_max = (overlap.end().as_uvec3() - node_origin - UVec3::ONE) / UVec3::splat(cell_size);

		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as u8;
					let child_origin = node_origin + UVec3::new(x, y, z) * cell_size;
					let child_region = NonZeroVoxelRegion::from_min_size(child_origin.as_ivec3(), UVec3::splat(cell_size)).unwrap();
					if region.contains_region(child_region) || node_depth == 0 {
						self.set_child_area_to_data(node_index, node_depth, child_index, data);
						continue;
					}
					let child_node_index = match self.child_node_for_partial_area(node_index, child_index) {
						Some(index) => index,
						None => return false,
					};
					if !self.fill_region_recurse(child_node_index, node_depth - 1, child_origin, region, data) {
						return false;
					}
					self.collapse_child_node_if_possible(node_index, child_index);
				}
			}
		}
		true
	}

	fn occupied_count_in_region(&self, region: NonZeroVoxelRegion) -> u64 {
		if self.is_empty() {
			return 0;
		}
		let root_region = self.root_region();
		let Some(region) = root_region.intersection(region) else { return 0 };
		if region == root_region {
			return self.raw.item_count();
		}

		fn recurse<G: GridType>(tree: &GridTree64<G>, node_index: u32, node_depth: u8, node_origin: UVec3, region: NonZeroVoxelRegion) -> u64 {
			if tree.raw.used_cell_count(node_index) == 0 {
				return 0;
			}

			let node_region = NonZeroVoxelRegion::from_min_size(node_origin.as_ivec3(), UVec3::splat(size(node_depth))).unwrap();
			if region == node_region {
				return tree.occupied_count_in_node(node_index, node_depth);
			}

			let cell_size = child_size(node_depth);
			let child_min = (region.min().as_uvec3() - node_origin) / UVec3::splat(cell_size);
			let child_max = (region.end().as_uvec3() - node_origin - UVec3::ONE) / UVec3::splat(cell_size);
			let mut count = 0u64;
			for z in child_min.z..=child_max.z {
				for y in child_min.y..=child_max.y {
					for x in child_min.x..=child_max.x {
						let child_index = (x + y * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as u8;
						let cell_kind = tree.raw.cell_kind(node_index, child_index);
						if cell_kind == CellKind::Empty {
							continue;
						}

						let child_origin = node_origin + UVec3::new(x, y, z) * cell_size;
						let child_region = NonZeroVoxelRegion::from_min_size(child_origin.as_ivec3(), UVec3::splat(cell_size)).unwrap();
						if region.contains_region(child_region) {
							count += tree.occupied_count_in_cell(node_depth, node_index, child_index);
							continue;
						}

						let overlap = NonZeroVoxelRegion::from_min_end(child_region.min().max(region.min()), child_region.end().min(region.end())).unwrap();
						match cell_kind {
							CellKind::Empty => unreachable!(),
							CellKind::Data => {
								let size = overlap.size();
								count += size.x as u64 * size.y as u64 * size.z as u64;
							}
							CellKind::Node => {
								count += if node_depth == 0 { 1 } else { recurse(tree, tree.raw.child_index(node_index, child_index), node_depth - 1, child_origin, overlap) };
							}
						}
					}
				}
			}
			count
		}

		recurse(self, 0, self.raw.root_depth(), self.raw.root_pos(), region)
	}

	fn clear_destination_covered_by_source(&mut self, source: &Self, source_region: NonZeroVoxelRegion, offset: IVec3) {
		self.clear_destination_covered_by_source_recurse(source, 0, source.raw.root_depth(), source.raw.root_pos(), source_region, offset);
	}

	fn clear_destination_covered_by_source_recurse(
		&mut self,
		source: &Self,
		src_node_index: u32,
		src_node_depth: u8,
		src_node_origin: UVec3,
		source_region: NonZeroVoxelRegion,
		offset: IVec3,
	) {
		let node_region = NonZeroVoxelRegion::from_min_size(src_node_origin.as_ivec3(), UVec3::splat(size(src_node_depth))).unwrap();
		let Some(overlap) = source_region.intersection(node_region) else { return };
		let cell_size = child_size(src_node_depth);
		let child_min = (overlap.min().as_uvec3() - src_node_origin) / UVec3::splat(cell_size);
		let child_max = (overlap.end().as_uvec3() - src_node_origin - UVec3::ONE) / UVec3::splat(cell_size);
		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as u8;
					let cell_kind = source.raw.cell_kind(src_node_index, child_index);
					if cell_kind == CellKind::Empty {
						continue;
					}

					let child_origin = src_node_origin + UVec3::new(x, y, z) * cell_size;
					let child_region = NonZeroVoxelRegion::from_min_size(child_origin.as_ivec3(), UVec3::splat(cell_size)).unwrap();
					let clipped_source = NonZeroVoxelRegion::from_min_end(child_region.min().max(overlap.min()), child_region.end().min(overlap.end())).unwrap();
					match cell_kind {
						CellKind::Empty => unreachable!(),
						CellKind::Data => self.clear_region(clipped_source.translated(offset)),
						CellKind::Node => {
							if src_node_depth == 0 {
								self.clear_region(clipped_source.translated(offset));
							} else {
								self.clear_destination_covered_by_source_recurse(source, source.raw.child_index(src_node_index, child_index), src_node_depth - 1, child_origin, clipped_source, offset);
							}
						}
					}
				}
			}
		}
	}

	pub fn clear_region(&mut self, region: NonZeroVoxelRegion) {
		if self.is_empty() {
			return;
		}
		let Some(region) = self.root_region().intersection(region) else { return };
		if !self.has_node_budget() {
			return;
		}

		let _ = self.clear_region_recurse(0, self.raw.root_depth(), self.raw.root_pos(), region);
	}

	fn clear_region_outside(&mut self, region: NonZeroVoxelRegion, keep: NonZeroVoxelRegion) {
		let mut remaining_min = region.min();
		let mut remaining_end = region.end();
		for axis in 0..3 {
			if keep.min()[axis] > remaining_min[axis] {
				let mut slab_end = remaining_end;
				slab_end[axis] = keep.min()[axis];
				self.clear_region(NonZeroVoxelRegion::from_min_end(remaining_min, slab_end).unwrap());
				remaining_min[axis] = keep.min()[axis];
			}
			if keep.end()[axis] < remaining_end[axis] {
				let mut slab_min = remaining_min;
				slab_min[axis] = keep.end()[axis];
				self.clear_region(NonZeroVoxelRegion::from_min_end(slab_min, remaining_end).unwrap());
				remaining_end[axis] = keep.end()[axis];
			}
		}
	}

	pub fn remove_area(&mut self, pos: &UVec3, size: UVec3) {
		let Some(region) = NonZeroVoxelRegion::from_min_size(pos.as_ivec3(), size) else { return };
		self.clear_region(region);
	}

	fn root_region(&self) -> NonZeroVoxelRegion {
		NonZeroVoxelRegion::from_min_size(self.raw.root_pos().as_ivec3(), UVec3::splat(size(self.raw.root_depth()))).unwrap()
	}

	pub fn occupied_bounds(&self) -> Option<NonZeroVoxelRegion> {
		(!self.is_empty()).then(|| self.root_region()).and_then(|region| self.occupied_bounds_in_region(region))
	}

	pub fn occupied_bounds_in_region(&self, region: NonZeroVoxelRegion) -> Option<NonZeroVoxelRegion> {
		#[derive(Clone, Copy)]
		enum Axis { X, Y, Z }

		#[derive(Clone, Copy, PartialEq, Eq)]
		enum Edge { Min, Max }

		#[inline]
		fn axis_value(v: UVec3, axis: Axis) -> u32 {
			match axis { Axis::X => v.x, Axis::Y => v.y, Axis::Z => v.z }
		}

		#[inline]
		fn region_edge(region: NonZeroVoxelRegion, axis: Axis, edge: Edge) -> u32 {
			match (axis, edge) {
				(Axis::X, Edge::Min) => region.min().x as u32,
				(Axis::X, Edge::Max) => region.end().x as u32,
				(Axis::Y, Edge::Min) => region.min().y as u32,
				(Axis::Y, Edge::Max) => region.end().y as u32,
				(Axis::Z, Edge::Min) => region.min().z as u32,
				(Axis::Z, Edge::Max) => region.end().z as u32,
			}
		}

		fn frontier_from_child<G: GridType>(
			tree: &GridTree64<G>,
			node_index: u32,
			child_index: u8,
			node_depth: u8,
			child_origin: UVec3,
			cell_size: u32,
			region: NonZeroVoxelRegion,
			axis: Axis,
			edge: Edge,
		) -> Option<u32> {
			let child_region = NonZeroVoxelRegion::from_min_size(child_origin.as_ivec3(), UVec3::splat(cell_size)).unwrap();
			let overlap = if region.contains_region(child_region) {
				child_region
			} else {
				NonZeroVoxelRegion::from_min_end(child_region.min().max(region.min()), child_region.end().min(region.end())).unwrap()
			};
			match tree.raw.cell_kind(node_index, child_index) {
				CellKind::Empty => None,
				CellKind::Data => Some(region_edge(overlap, axis, edge)),
				CellKind::Node if node_depth == 0 => Some(region_edge(child_region, axis, edge)),
				CellKind::Node => frontier(tree, tree.raw.child_index(node_index, child_index), node_depth - 1, child_origin, overlap, axis, edge),
			}
		}

		fn frontier<G: GridType>(
			tree: &GridTree64<G>,
			node_index: u32,
			node_depth: u8,
			node_origin: UVec3,
			region: NonZeroVoxelRegion,
			axis: Axis,
			edge: Edge,
		) -> Option<u32> {
			if tree.raw.used_cell_count(node_index) == 0 {
				return None;
			}

			let cell_size = child_size(node_depth);
			let child_min = (region.min().as_uvec3() - node_origin) / UVec3::splat(cell_size);
			let child_max = (region.end().as_uvec3() - node_origin - UVec3::ONE) / UVec3::splat(cell_size);
			let outer_start = axis_value(child_min, axis);
			let outer_end = axis_value(child_max, axis);
			let region_min = region_edge(region, axis, Edge::Min);
			let region_max = region_edge(region, axis, Edge::Max);

			let mut outer = if edge == Edge::Min { outer_start } else { outer_end };
			loop {
				let slab_min = axis_value(node_origin, axis) + outer * cell_size;
				let slab_end = slab_min + cell_size;
				let best_possible = if edge == Edge::Min { slab_min.max(region_min) } else { slab_end.min(region_max) };
				let mut best: Option<u32> = None;

				match axis {
					Axis::X => {
						for z in child_min.z..=child_max.z {
							for y in child_min.y..=child_max.y {
								let child_index = (outer + y * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as u8;
								if tree.raw.cell_kind(node_index, child_index) == CellKind::Empty { continue; }
								let child_origin = node_origin + UVec3::new(outer, y, z) * cell_size;
								if let Some(candidate) = frontier_from_child(tree, node_index, child_index, node_depth, child_origin, cell_size, region, axis, edge) {
									best = Some(match best { Some(current) if edge == Edge::Min => current.min(candidate), Some(current) => current.max(candidate), None => candidate });
									if Some(best_possible) == best { return best; }
								}
							}
						}
					}
					Axis::Y => {
						for z in child_min.z..=child_max.z {
							for x in child_min.x..=child_max.x {
								let child_index = (x + outer * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as u8;
								if tree.raw.cell_kind(node_index, child_index) == CellKind::Empty { continue; }
								let child_origin = node_origin + UVec3::new(x, outer, z) * cell_size;
								if let Some(candidate) = frontier_from_child(tree, node_index, child_index, node_depth, child_origin, cell_size, region, axis, edge) {
									best = Some(match best { Some(current) if edge == Edge::Min => current.min(candidate), Some(current) => current.max(candidate), None => candidate });
									if Some(best_possible) == best { return best; }
								}
							}
						}
					}
					Axis::Z => {
						for y in child_min.y..=child_max.y {
							for x in child_min.x..=child_max.x {
								let child_index = (x + y * SIZE as u32 + outer * SIZE as u32 * SIZE as u32) as u8;
								if tree.raw.cell_kind(node_index, child_index) == CellKind::Empty { continue; }
								let child_origin = node_origin + UVec3::new(x, y, outer) * cell_size;
								if let Some(candidate) = frontier_from_child(tree, node_index, child_index, node_depth, child_origin, cell_size, region, axis, edge) {
									best = Some(match best { Some(current) if edge == Edge::Min => current.min(candidate), Some(current) => current.max(candidate), None => candidate });
									if Some(best_possible) == best { return best; }
								}
							}
						}
					}
				}

				if best.is_some() { return best; }
				if edge == Edge::Min {
					if outer == outer_end { break; }
					outer += 1;
				} else {
					if outer == outer_start { break; }
					outer -= 1;
				}
			}
			None
		}

		let region = self.root_region().intersection(region)?;
		NonZeroVoxelRegion::from_min_end(
			UVec3::new(
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::X, Edge::Min)?,
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::Y, Edge::Min)?,
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::Z, Edge::Min)?,
			).as_ivec3(),
			UVec3::new(
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::X, Edge::Max)?,
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::Y, Edge::Max)?,
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::Z, Edge::Max)?,
			).as_ivec3(),
		)
	}

	pub(super) fn clear_region_recurse(&mut self, node_index: u32, node_depth: u8, node_origin: UVec3, region: NonZeroVoxelRegion) -> bool {
		let Some(overlap) = region.intersection(NonZeroVoxelRegion::from_min_size(node_origin.as_ivec3(), UVec3::splat(size(node_depth))).unwrap()) else { return true };
		let cell_size = child_size(node_depth) as u32;
		let child_min = (overlap.min().as_uvec3() - node_origin) / UVec3::splat(cell_size);
		let child_max = (overlap.end().as_uvec3() - node_origin - UVec3::ONE) / UVec3::splat(cell_size);
		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as u8;
					let child_origin = node_origin + UVec3::new(x, y, z) * cell_size;
					let child_region = NonZeroVoxelRegion::from_min_size(child_origin.as_ivec3(), UVec3::splat(cell_size)).unwrap();
					let fully_covered = region.contains(child_region.min()) && region.contains(child_region.end() - IVec3::ONE);

					if fully_covered || node_depth == 0 {
						self.set_child_area_to_empty(node_index, node_depth, child_index);
						continue;
					}

					if self.raw.cell_kind(node_index, child_index) == CellKind::Empty {
						continue;
					}
					let child_node_index = match self.child_node_for_partial_area(node_index, child_index) {
						Some(index) => index,
						None => return false,
					};
					if !self.clear_region_recurse(child_node_index, node_depth - 1, child_origin, region) {
						return false;
					}
					self.collapse_child_node_if_possible(node_index, child_index);
				}
			}
		}
		true
	}
}
