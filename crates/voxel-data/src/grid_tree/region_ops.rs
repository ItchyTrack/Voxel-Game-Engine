use bevy::math::{IVec2, IVec3, Vec3};

use crate::sdf::{shrink_aabb_with_sdf, voxel_center, voxel_region_from_bounds, Sdf};

use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BlockSdfRelation {
	Outside,
	Inside,
	Intersecting,
}

fn sdf_relation_for_block(sdf: &(impl Sdf + ?Sized), block_min: IVec3, block_size: i32) -> BlockSdfRelation {
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

impl<G: GridType, Co: GridCoord> GridTree<G, Co> {
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

	fn fill_sdf_region(&mut self, region: GridRegion, sdf: &(impl Sdf + ?Sized), data: G::Data<'_>) {
		let Some(region) = GridRegion::new(region.min, region.end) else { return };
		if !self.make_sure_root_covers_area(region.min, region.max_inclusive()) || !self.has_node_budget() {
			return;
		}

		let _ = self.fill_sdf_recurse(0, self.raw.root_depth(), self.raw.root_pos(), region, sdf, data);
	}

	fn clear_sdf_region(&mut self, region: GridRegion, sdf: &(impl Sdf + ?Sized)) {
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
		node_origin: IVec3,
		region: GridRegion,
		sdf: &(impl Sdf + ?Sized),
		data: G::Data<'_>,
	) -> bool {
		let node_region = GridRegion { min: node_origin, end: node_origin + IVec3::splat(size(node_depth) as i32) };
		let Some(overlap) = node_region.intersection(region) else { return true };
		if region.contains_region(node_region) {
			match sdf_relation_for_block(sdf, node_origin, size(node_depth) as i32) {
				BlockSdfRelation::Outside => return true,
				BlockSdfRelation::Inside => {
					if node_index == 0 {
						for z in 0..SIZE as i32 {
							for y in 0..SIZE as i32 {
								for x in 0..SIZE as i32 {
									let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
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
		let cell_size = child_size(node_depth) as i32;
		let child_min = (overlap.min - node_origin).div_euclid(IVec3::splat(cell_size));
		let child_max = (overlap.end - node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));

		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
					let child_origin = node_origin + IVec3::new(x, y, z) * cell_size;
					let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
					match sdf_relation_for_block(sdf, child_origin, cell_size) {
						BlockSdfRelation::Outside => continue,
						BlockSdfRelation::Inside if region.contains_region(child_region) => {
							self.set_child_area_to_data(node_index, node_depth, child_index, data);
							continue;
						}
						BlockSdfRelation::Inside | BlockSdfRelation::Intersecting => {}
					}
					if node_depth == 0 {
						if sdf.sample(voxel_center(child_origin)) <= 0.0 {
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
		node_origin: IVec3,
		region: GridRegion,
		sdf: &(impl Sdf + ?Sized),
	) -> bool {
		let node_region = GridRegion { min: node_origin, end: node_origin + IVec3::splat(size(node_depth) as i32) };
		let Some(overlap) = node_region.intersection(region) else { return true };
		if region.contains_region(node_region) {
			match sdf_relation_for_block(sdf, node_origin, size(node_depth) as i32) {
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
		let cell_size = child_size(node_depth) as i32;
		let child_min = (overlap.min - node_origin).div_euclid(IVec3::splat(cell_size));
		let child_max = (overlap.end - node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));

		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
					let child_origin = node_origin + IVec3::new(x, y, z) * cell_size;
					let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
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
						if sdf.sample(voxel_center(child_origin)) <= 0.0 {
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

	pub fn split_region(&mut self, region: GridRegion) -> Self {
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

	pub fn merge_region_from(&mut self, other: &Self, source_region: GridRegion, offset: IVec3) {
		if other.is_empty() {
			return;
		}
		let Some(source_region) = source_region.intersection(other.root_region()) else { return };
		if self.is_empty() && source_region.contains_region(other.root_region()) {
			*self = other.clone();
			self.raw.set_root(other.raw.root_pos() + offset, other.raw.root_depth());
			return;
		}
		let Some(source_bounds) = other.occupied_bounds_in_region(source_region) else { return };
		self.merge_region_from_with_bounds(other, source_region, source_bounds, offset);
	}

	pub fn merge_region_from_mapped<'a>(&'a mut self, other: &'a Self, source_region: GridRegion, offset: IVec3, mut map: impl FnMut(G::Data<'a>) -> G::Data<'a>) {
		// Keep the mapped API, but mapping arbitrary borrowed GridData cannot use the
		// byte-copy aligned-node fast path. The identity merge path above is the hot
		// voxel merge path and preserves the old structural algorithm.
		if other.is_empty() { return; }
		let Some(source_region) = source_region.intersection(other.root_region()) else { return };
		let Some(source_bounds) = other.occupied_bounds_in_region(source_region) else { return };
		let dest_bounds = source_bounds.translated(offset);
		if !self.make_sure_root_covers_area(dest_bounds.min, dest_bounds.max_inclusive()) { return; }
		let leaves: Vec<_> = other.view().leaves().filter_map(|leaf| {
			let leaf_region = GridRegion { min: leaf.origin, end: leaf.origin + IVec3::splat(leaf.size as i32) };
			leaf_region.intersection(source_region).map(|clipped| (clipped, leaf.data_value()))
		}).collect();
		for (clipped, data) in leaves {
			self.fill_region(clipped.translated(offset), map(data));
		}
	}

	fn merge_region_from_with_bounds(&mut self, other: &Self, source_region: GridRegion, source_bounds: GridRegion, offset: IVec3) {
		let source_count = other.occupied_count_in_region(source_bounds);
		let walk_destination = !self.is_empty() && self.len() < source_count;
		if walk_destination {
			self.clear_destination_covered_by_source(other, source_bounds, offset);
		}
		self.merge_region_from_source_walk(other, source_bounds, source_region.contains_region(other.root_region()), offset);
	}

	fn merge_region_from_source_walk(&mut self, other: &Self, source_bounds: GridRegion, full_root_covered: bool, offset: IVec3) {
		let dest_bounds = source_bounds.translated(offset);
		if !self.make_sure_root_covers_area(dest_bounds.min, dest_bounds.max_inclusive()) || !self.has_node_budget() {
			return;
		}

		if full_root_covered {
			if let Some(dest_node_index) = self.node_for_region(other.raw.root_pos() + offset, other.raw.root_depth()) {
				if self.merge_aligned_nodes_from(other, 0, dest_node_index, other.raw.root_depth()) {
					return;
				}
			}
		}
		let _ = self.merge_region_from_recurse(other, 0, other.raw.root_depth(), other.raw.root_pos(), source_bounds, offset);
	}

	fn node_for_region(&mut self, target_origin: IVec3, target_depth: u8) -> Option<u32> {
		let mut node_index = 0;
		let mut node_depth = self.raw.root_depth();
		let mut node_origin = self.raw.root_pos();
		while node_depth > target_depth {
			let cell_size = child_size(node_depth) as i32;
			let rel = target_origin - node_origin;
			if rel.is_negative_bitmask() != 0 {
				return None;
			}
			let child_pos = rel.div_euclid(IVec3::splat(cell_size));
			if child_pos.cmplt(IVec3::ZERO).any() || child_pos.cmpge(IVec3::splat(SIZE as i32)).any() {
				return None;
			}
			let child_index = (child_pos.x + child_pos.y * SIZE as i32 + child_pos.z * SIZE as i32 * SIZE as i32) as u8;
			node_origin += child_pos * cell_size;
			node_index = self.child_node_for_partial_area(node_index, child_index)?;
			node_depth -= 1;
		}
		(node_origin == target_origin && node_depth == target_depth).then_some(node_index)
	}

	fn merge_aligned_nodes_from(&mut self, other: &Self, src_node_index: u32, dest_node_index: u32, node_depth: u8) -> bool {
		for child_index in 0..SIZE_CUBED {
			match other.raw.cell_kind(src_node_index, child_index) {
				CellKind::Empty => {}
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
					if !self.merge_aligned_nodes_from(other, other.raw.child_index(src_node_index, child_index), dest_child_node, node_depth - 1) {
						return false;
					}
					self.collapse_child_node_if_possible(dest_node_index, child_index);
				}
			}
		}
		true
	}

	fn merge_region_from_recurse(&mut self, other: &Self, src_node_index: u32, src_node_depth: u8, src_node_origin: IVec3, source_region: GridRegion, offset: IVec3) -> bool {
		let node_region = GridRegion { min: src_node_origin, end: src_node_origin + IVec3::splat(size(src_node_depth) as i32) };
		let Some(overlap) = source_region.intersection(node_region) else { return true };
		let cell_size = child_size(src_node_depth) as i32;
		let child_min = (overlap.min - src_node_origin).div_euclid(IVec3::splat(cell_size));
		let child_max = (overlap.end - src_node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));

		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
					let cell_kind = other.raw.cell_kind(src_node_index, child_index);
					if cell_kind == CellKind::Empty { continue; }
					let child_origin = src_node_origin + IVec3::new(x, y, z) * cell_size;
					let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
					let Some(clipped_source) = source_region.intersection(child_region) else { continue };
					match cell_kind {
						CellKind::Empty => unreachable!(),
						CellKind::Data => {
							let data = other.cell_data(src_node_index, child_index);
							if !self.fill_region_recurse_entry(clipped_source.translated(offset), data) { return false; }
						}
						CellKind::Node => {
							let child_node_index = other.raw.child_index(src_node_index, child_index);
							let child_depth = src_node_depth - 1;
							if source_region.contains_region(child_region) {
								if let Some(dest_node_index) = self.node_for_region(child_origin + offset, child_depth) {
									if !self.merge_aligned_nodes_from(other, child_node_index, dest_node_index, child_depth) { return false; }
									continue;
								}
							}
							if !self.merge_region_from_recurse(other, child_node_index, child_depth, child_origin, source_region, offset) { return false; }
						}
					}
				}
			}
		}
		true
	}

	fn fill_region_recurse_entry(&mut self, region: GridRegion, data: G::Data<'_>) -> bool {
		self.fill_region_recurse(0, self.raw.root_depth(), self.raw.root_pos(), region, data)
	}

	fn fill_region_recurse(&mut self, node_index: u32, node_depth: u8, node_origin: IVec3, region: GridRegion, data: G::Data<'_>) -> bool {
		let node_region = GridRegion { min: node_origin, end: node_origin + IVec3::splat(size(node_depth) as i32) };
		let Some(overlap) = region.intersection(node_region) else { return true };
		let cell_size = child_size(node_depth) as i32;
		let child_min = (overlap.min - node_origin).div_euclid(IVec3::splat(cell_size));
		let child_max = (overlap.end - node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));

		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
					let child_origin = node_origin + IVec3::new(x, y, z) * cell_size;
					let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
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

	fn occupied_count_in_region(&self, region: GridRegion) -> u64 {
		if self.is_empty() {
			return 0;
		}
		let root_region = self.root_region();
		let Some(region) = root_region.intersection(region) else { return 0 };
		if region == root_region {
			return self.raw.item_count();
		}

		fn recurse<G: GridType, Co: GridCoord>(tree: &GridTree<G, Co>, node_index: u32, node_depth: u8, node_origin: IVec3, region: GridRegion) -> u64 {
			if tree.raw.used_cell_count(node_index) == 0 {
				return 0;
			}

			let node_region = GridRegion { min: node_origin, end: node_origin + IVec3::splat(size(node_depth) as i32) };
			if region == node_region {
				return tree.occupied_count_in_node(node_index, node_depth);
			}

			let cell_size = child_size(node_depth) as i32;
			let child_min = (region.min - node_origin).div_euclid(IVec3::splat(cell_size));
			let child_max = (region.end - node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));
			let mut count = 0u64;
			for z in child_min.z..=child_max.z {
				for y in child_min.y..=child_max.y {
					for x in child_min.x..=child_max.x {
						let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
						let cell_kind = tree.raw.cell_kind(node_index, child_index);
						if cell_kind == CellKind::Empty {
							continue;
						}

						let child_origin = node_origin + IVec3::new(x, y, z) * cell_size;
						let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
						if region.contains_region(child_region) {
							count += tree.occupied_count_in_cell(node_depth, node_index, child_index);
							continue;
						}

						let overlap = GridRegion { min: child_region.min.max(region.min), end: child_region.end.min(region.end) };
						match cell_kind {
							CellKind::Empty => unreachable!(),
							CellKind::Data => {
								let size = overlap.size().as_uvec3();
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

	fn clear_destination_covered_by_source(&mut self, source: &Self, source_region: GridRegion, offset: IVec3) {
		self.clear_destination_covered_by_source_recurse(source, 0, source.raw.root_depth(), source.raw.root_pos(), source_region, offset);
	}

	fn clear_destination_covered_by_source_recurse(
		&mut self,
		source: &Self,
		src_node_index: u32,
		src_node_depth: u8,
		src_node_origin: IVec3,
		source_region: GridRegion,
		offset: IVec3,
	) {
		let node_region = GridRegion { min: src_node_origin, end: src_node_origin + IVec3::splat(size(src_node_depth) as i32) };
		let Some(overlap) = source_region.intersection(node_region) else { return };
		let cell_size = child_size(src_node_depth) as i32;
		let child_min = (overlap.min - src_node_origin).div_euclid(IVec3::splat(cell_size));
		let child_max = (overlap.end - src_node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));
		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
					let cell_kind = source.raw.cell_kind(src_node_index, child_index);
					if cell_kind == CellKind::Empty {
						continue;
					}

					let child_origin = src_node_origin + IVec3::new(x, y, z) * cell_size;
					let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
					let clipped_source = GridRegion { min: child_region.min.max(overlap.min), end: child_region.end.min(overlap.end) };
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

	pub fn clear_region(&mut self, region: GridRegion) {
		if self.is_empty() {
			return;
		}
		let Some(region) = self.root_region().intersection(region) else { return };
		if !self.has_node_budget() {
			return;
		}

		let _ = self.clear_region_recurse(0, self.raw.root_depth(), self.raw.root_pos(), region);
	}

	pub fn remove_area(&mut self, pos: &Co::Pos, size: IVec3) {
		let Some(region) = GridRegion::from_min_size(Co::to_ivec3(*pos), size) else { return };
		self.clear_region(region);
	}

	fn root_region(&self) -> GridRegion {
		GridRegion { min: self.raw.root_pos(), end: self.raw.root_pos() + IVec3::splat(size(self.raw.root_depth()) as i32) }
	}

	pub fn occupied_bounds(&self) -> Option<GridRegion> {
		(!self.is_empty()).then(|| self.root_region()).and_then(|region| self.occupied_bounds_in_region(region))
	}

	pub fn occupied_bounds_in_region(&self, region: GridRegion) -> Option<GridRegion> {
		#[derive(Clone, Copy)]
		enum Axis { X, Y, Z }

		#[derive(Clone, Copy, PartialEq, Eq)]
		enum Edge { Min, Max }

		#[inline]
		fn axis_value(v: IVec3, axis: Axis) -> i32 {
			match axis { Axis::X => v.x, Axis::Y => v.y, Axis::Z => v.z }
		}

		#[inline]
		fn region_edge(region: GridRegion, axis: Axis, edge: Edge) -> i32 {
			match (axis, edge) {
				(Axis::X, Edge::Min) => region.min.x,
				(Axis::X, Edge::Max) => region.end.x,
				(Axis::Y, Edge::Min) => region.min.y,
				(Axis::Y, Edge::Max) => region.end.y,
				(Axis::Z, Edge::Min) => region.min.z,
				(Axis::Z, Edge::Max) => region.end.z,
			}
		}

		fn frontier_from_child<G: GridType, Co: GridCoord>(
			tree: &GridTree<G, Co>,
			node_index: u32,
			child_index: u8,
			node_depth: u8,
			child_origin: IVec3,
			cell_size: i32,
			region: GridRegion,
			axis: Axis,
			edge: Edge,
		) -> Option<i32> {
			let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
			let overlap = if region.contains_region(child_region) {
				child_region
			} else {
				GridRegion { min: child_region.min.max(region.min), end: child_region.end.min(region.end) }
			};
			match tree.raw.cell_kind(node_index, child_index) {
				CellKind::Empty => None,
				CellKind::Data => Some(region_edge(overlap, axis, edge)),
				CellKind::Node if node_depth == 0 => Some(region_edge(child_region, axis, edge)),
				CellKind::Node => frontier(tree, tree.raw.child_index(node_index, child_index), node_depth - 1, child_origin, overlap, axis, edge),
			}
		}

		fn frontier<G: GridType, Co: GridCoord>(
			tree: &GridTree<G, Co>,
			node_index: u32,
			node_depth: u8,
			node_origin: IVec3,
			region: GridRegion,
			axis: Axis,
			edge: Edge,
		) -> Option<i32> {
			if tree.raw.used_cell_count(node_index) == 0 {
				return None;
			}

			let cell_size = child_size(node_depth) as i32;
			let child_min = (region.min - node_origin).div_euclid(IVec3::splat(cell_size));
			let child_max = (region.end - node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));
			let outer_start = axis_value(child_min, axis);
			let outer_end = axis_value(child_max, axis);
			let region_min = region_edge(region, axis, Edge::Min);
			let region_max = region_edge(region, axis, Edge::Max);

			let mut outer = if edge == Edge::Min { outer_start } else { outer_end };
			loop {
				let slab_min = axis_value(node_origin, axis) + outer * cell_size;
				let slab_end = slab_min + cell_size;
				let best_possible = if edge == Edge::Min { slab_min.max(region_min) } else { slab_end.min(region_max) };
				let mut best: Option<i32> = None;

				match axis {
					Axis::X => {
						for z in child_min.z..=child_max.z {
							for y in child_min.y..=child_max.y {
								let child_index = (outer + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
								if tree.raw.cell_kind(node_index, child_index) == CellKind::Empty { continue; }
								let child_origin = node_origin + IVec3::new(outer, y, z) * cell_size;
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
								let child_index = (x + outer * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
								if tree.raw.cell_kind(node_index, child_index) == CellKind::Empty { continue; }
								let child_origin = node_origin + IVec3::new(x, outer, z) * cell_size;
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
								let child_index = (x + y * SIZE as i32 + outer * SIZE as i32 * SIZE as i32) as u8;
								if tree.raw.cell_kind(node_index, child_index) == CellKind::Empty { continue; }
								let child_origin = node_origin + IVec3::new(x, y, outer) * cell_size;
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
		Some(GridRegion {
			min: IVec3::new(
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::X, Edge::Min)?,
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::Y, Edge::Min)?,
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::Z, Edge::Min)?,
			),
			end: IVec3::new(
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::X, Edge::Max)?,
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::Y, Edge::Max)?,
				frontier(self, 0, self.raw.root_depth(), self.raw.root_pos(), region, Axis::Z, Edge::Max)?,
			),
		})
	}

	pub(super) fn clear_region_recurse(&mut self, node_index: u32, node_depth: u8, node_origin: IVec3, region: GridRegion) -> bool {
		let Some(overlap) = region.intersection(GridRegion { min: node_origin, end: node_origin + IVec3::splat(size(node_depth) as i32) }) else { return true };
		let cell_size = child_size(node_depth) as i32;
		let child_min = (overlap.min - node_origin).div_euclid(IVec3::splat(cell_size));
		let child_max = (overlap.end - node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));
		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
					let child_origin = node_origin + IVec3::new(x, y, z) * cell_size;
					let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
					let fully_covered = region.contains(child_region.min) && region.contains(child_region.end - IVec3::ONE);

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
