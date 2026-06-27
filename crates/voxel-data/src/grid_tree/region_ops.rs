use bevy::math::{IVec2, Vec3};

use crate::sdf::{shrink_aabb_with_sdf, voxel_center, voxel_region_from_bounds, Sdf};

use super::*;

impl<C: GridCell, Co: GridCoord> GridTree<C, Co> {
	pub fn apply_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize, data: C::Data) {
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, sdf, face_resolution, iterations);
		let Some(region) = voxel_region_from_bounds(min, max) else { return };
		self.fill_sdf_region(region, sdf, data);
	}

	pub fn clear_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize) {
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, sdf, face_resolution, iterations);
		let Some(region) = voxel_region_from_bounds(min, max) else { return };
		self.clear_sdf_region(region, sdf);
	}

	fn fill_sdf_region(&mut self, region: GridRegion, sdf: &(impl Sdf + ?Sized), data: C::Data) {
		let Some(region) = GridRegion::new(region.min, region.end) else { return };
		if !self.make_sure_root_covers_area(region.min, region.max_inclusive()) || !self.has_node_budget() {
			return;
		}

		for attempt in 0..3 {
			if self.fill_sdf_recurse(0, self.root_depth, self.root_pos, region, sdf, data) {
				return;
			}
			if attempt < 2 {
				self.compact();
			}
		}
		bevy::log::warn!("GridTree could not finish apply_sdf after compaction retries");
	}

	fn clear_sdf_region(&mut self, region: GridRegion, sdf: &(impl Sdf + ?Sized)) {
		if self.is_empty() {
			return;
		}
		let Some(region) = self.root_region().intersection(region) else { return };
		if !self.has_node_budget() {
			return;
		}

		for attempt in 0..3 {
			if self.clear_sdf_recurse(0, self.root_depth, self.root_pos, region, sdf) {
				return;
			}
			if attempt < 2 {
				self.compact();
			}
		}
		bevy::log::warn!("GridTree could not finish clear_sdf after compaction retries");
	}

	fn fill_sdf_recurse(
		&mut self,
		node_index: u32,
		node_depth: u8,
		node_origin: IVec3,
		region: GridRegion,
		sdf: &(impl Sdf + ?Sized),
		data: C::Data,
	) -> bool {
		let node_region = GridRegion { min: node_origin, end: node_origin + IVec3::splat(size(node_depth) as i32) };
		let Some(overlap) = node_region.intersection(region) else { return true };
		let cell_size = child_size(node_depth) as i32;
		let child_min = (overlap.min - node_origin).div_euclid(IVec3::splat(cell_size));
		let child_max = (overlap.end - node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));

		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
					let child_origin = node_origin + IVec3::new(x, y, z) * cell_size;
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
		let cell_size = child_size(node_depth) as i32;
		let child_min = (overlap.min - node_origin).div_euclid(IVec3::splat(cell_size));
		let child_max = (overlap.end - node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));

		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
					let child_origin = node_origin + IVec3::new(x, y, z) * cell_size;
					let cell = self.nodes[node_index as usize].get_child_cell_from_index(child_index);
					if cell.kind() == CellKind::Empty {
						continue;
					}
					if node_depth == 0 {
						if sdf.sample(voxel_center(child_origin)) <= 0.0 {
							self.set_child_area_to_empty(node_index, node_depth, child_index);
						}
						continue;
					}

					if cell.kind() != CellKind::Node {
						let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
						if !region.intersects(child_region) {
							continue;
						}
					}

					let child_node_index = match cell.kind() {
						CellKind::Node => node_index + cell.node_offset(),
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
		let mut out = Self::new();
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
		let Some(_source_bounds) = other.occupied_bounds_in_region(source_region) else { return };

		if self.is_empty() && source_region.contains_region(other.root_region()) {
			*self = other.clone();
			self.root_pos += offset;
			return;
		}

		self.merge_region_from_mapped(other, source_region, offset, |data| data);
	}

	pub fn merge_region_from_mapped(&mut self, other: &Self, source_region: GridRegion, offset: IVec3, mut map: impl FnMut(C::Data) -> C::Data) {
		if other.is_empty() {
			return;
		}
		let Some(source_region) = source_region.intersection(other.root_region()) else { return };
		if other.occupied_bounds_in_region(source_region).is_none() {
			return;
		}

		let source_count = other.occupied_count_in_region(source_region);
		let walk_destination = !self.is_empty() && self.len() < source_count;
		if walk_destination {
			self.clear_destination_covered_by_source(other, source_region, offset);
		}
		self.merge_region_from_mapped_source_walk(other, source_region, offset, &mut map);
	}

	fn merge_region_from_mapped_source_walk(&mut self, other: &Self, source_region: GridRegion, offset: IVec3, map: &mut impl FnMut(C::Data) -> C::Data) {
		let Some(source_bounds) = other.occupied_bounds_in_region(source_region) else { return };
		let dest_bounds = source_bounds.translated(offset);
		if !self.make_sure_root_covers_area(dest_bounds.min, dest_bounds.max_inclusive()) || !self.has_node_budget() {
			return;
		}

		for attempt in 0..3 {
			if source_region.contains_region(other.root_region()) {
				if let Some(dest_node_index) = self.node_for_region(other.root_pos + offset, other.root_depth) {
					if self.merge_aligned_nodes_from_mapped(other, 0, dest_node_index, other.root_depth, map) {
						return;
					}
				}
			}
			if self.merge_region_from_mapped_recurse(other, 0, other.root_depth, other.root_pos, source_region, offset, map) {
				return;
			}
			if attempt < 2 {
				self.compact();
			}
		}
		bevy::log::warn!("GridTree could not finish merge_region_from_mapped after compaction retries");
	}

	fn node_for_region(&mut self, target_origin: IVec3, target_depth: u8) -> Option<u32> {
		let mut node_index = 0;
		let mut node_depth = self.root_depth;
		let mut node_origin = self.root_pos;
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

	fn merge_aligned_nodes_from_mapped(
		&mut self,
		other: &Self,
		src_node_index: u32,
		dest_node_index: u32,
		node_depth: u8,
		map: &mut impl FnMut(C::Data) -> C::Data,
	) -> bool {
		for child_index in 0..SIZE_CUBED {
			let src_cell = other.nodes[src_node_index as usize].get_child_cell_from_index(child_index);
			match src_cell.kind() {
				CellKind::Empty => {}
				CellKind::Data => self.set_child_area_to_data(dest_node_index, node_depth, child_index, map(src_cell.data_value())),
				CellKind::Node => {
					if node_depth == 0 {
						continue;
					}
					let dest_child_node = match self.child_node_for_partial_area(dest_node_index, child_index) {
						Some(index) => index,
						None => return false,
					};
					if !self.merge_aligned_nodes_from_mapped(other, src_node_index + src_cell.node_offset(), dest_child_node, node_depth - 1, map) {
						return false;
					}
					self.collapse_child_node_if_possible(dest_node_index, child_index);
				}
			}
		}
		true
	}

	fn merge_region_from_mapped_recurse(
		&mut self,
		other: &Self,
		src_node_index: u32,
		src_node_depth: u8,
		src_node_origin: IVec3,
		source_region: GridRegion,
		offset: IVec3,
		map: &mut impl FnMut(C::Data) -> C::Data,
	) -> bool {
		let node_region = GridRegion { min: src_node_origin, end: src_node_origin + IVec3::splat(size(src_node_depth) as i32) };
		let Some(overlap) = source_region.intersection(node_region) else { return true };
		let cell_size = child_size(src_node_depth) as i32;
		let child_min = (overlap.min - src_node_origin).div_euclid(IVec3::splat(cell_size));
		let child_max = (overlap.end - src_node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));

		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
					let cell = other.nodes[src_node_index as usize].get_child_cell_from_index(child_index);
					if cell.kind() == CellKind::Empty {
						continue;
					}
					let child_origin = src_node_origin + IVec3::new(x, y, z) * cell_size;
					let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
					let Some(clipped_source) = source_region.intersection(child_region) else { continue };
					match cell.kind() {
						CellKind::Empty => unreachable!(),
						CellKind::Data => {
							let data = map(cell.data_value());
							if !self.fill_region_recurse_entry(clipped_source.translated(offset), data) {
								return false;
							}
						}
						CellKind::Node => {
							let child_node_index = src_node_index + cell.node_offset();
							let child_depth = src_node_depth - 1;
							if source_region.contains_region(child_region) {
								if let Some(dest_node_index) = self.node_for_region(child_origin + offset, child_depth) {
									if !self.merge_aligned_nodes_from_mapped(other, child_node_index, dest_node_index, child_depth, map) {
										return false;
									}
									continue;
								}
							}
							if !self.merge_region_from_mapped_recurse(other, child_node_index, child_depth, child_origin, source_region, offset, map) {
								return false;
							}
						}
					}
				}
			}
		}
		true
	}

	fn fill_region_recurse_entry(&mut self, region: GridRegion, data: C::Data) -> bool {
		self.fill_region_recurse(0, self.root_depth, self.root_pos, region, data)
	}

	fn fill_region_recurse(&mut self, node_index: u32, node_depth: u8, node_origin: IVec3, region: GridRegion, data: C::Data) -> bool {
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
		let mut count = 0u64;
		self.each_leaf_in_region(region, |origin, cell_size, _| {
			let leaf = GridRegion { min: origin, end: origin + IVec3::splat(cell_size as i32) };
			if let Some(overlap) = region.intersection(leaf) {
				let size = overlap.size().as_uvec3();
				count += size.x as u64 * size.y as u64 * size.z as u64;
			}
		});
		count
	}

	fn clear_destination_covered_by_source(&mut self, source: &Self, source_region: GridRegion, offset: IVec3) {
		let mut leaves = Vec::new();
		self.each_leaf(|origin, size, _| leaves.push(GridRegion { min: origin, end: origin + IVec3::splat(size as i32) }));
		for dest_leaf in leaves {
			self.clear_dest_leaf_covered_by_source(source, source_region, dest_leaf.translated(-offset), offset);
		}
	}

	fn clear_dest_leaf_covered_by_source(&mut self, source: &Self, source_region: GridRegion, region_in_source_space: GridRegion, offset: IVec3) {
		let Some(region) = region_in_source_space.intersection(source_region) else { return };
		if !source.any_in_region(region) {
			return;
		}
		if source.is_region_filled(region) {
			self.clear_region(region.translated(offset));
			return;
		}
		let sz = region.size();
		if sz == IVec3::ONE {
			self.clear_region(region.translated(offset));
			return;
		}
		let split = (sz / 2).max(IVec3::ONE);
		let mid = region.min + split;
		let xs = [(region.min.x, mid.x), (mid.x, region.end.x)];
		let ys = [(region.min.y, mid.y), (mid.y, region.end.y)];
		let zs = [(region.min.z, mid.z), (mid.z, region.end.z)];
		for (x0, x1) in xs {
			for (y0, y1) in ys {
				for (z0, z1) in zs {
					if let Some(child) = GridRegion::new(IVec3::new(x0, y0, z0), IVec3::new(x1, y1, z1)) {
						self.clear_dest_leaf_covered_by_source(source, source_region, child, offset);
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

		for attempt in 0..3 {
			if self.clear_region_recurse(0, self.root_depth, self.root_pos, region) {
				return;
			}
			if attempt < 2 {
				self.compact();
			}
		}
		bevy::log::warn!("GridTree could not finish clear_region after compaction retries");
	}

	pub fn remove_area(&mut self, pos: &Co::Pos, size: IVec3) {
		let Some(region) = GridRegion::from_min_size(Co::to_ivec3(*pos), size) else { return };
		self.clear_region(region);
	}

	fn root_region(&self) -> GridRegion {
		GridRegion { min: self.root_pos, end: self.root_pos + IVec3::splat(size(self.root_depth) as i32) }
	}

	fn occupied_bounds_in_region(&self, region: GridRegion) -> Option<GridRegion> {
		let mut bounds: Option<GridRegion> = None;
		self.each_leaf_in_region(region, |origin, cell_size, _| {
			let leaf = GridRegion { min: origin, end: origin + IVec3::splat(cell_size as i32) };
			let Some(overlap) = region.intersection(leaf) else { return };
			bounds = Some(match bounds {
				Some(existing) => GridRegion { min: existing.min.min(overlap.min), end: existing.end.max(overlap.end) },
				None => overlap,
			});
		});
		bounds
	}

	fn each_leaf_in_region(&self, region: GridRegion, mut f: impl FnMut(IVec3, u32, C::Data)) {
		let mut stack: Vec<(u32, u8, IVec3)> = vec![(0, self.root_depth, self.root_pos)];
		while let Some((node_index, depth, origin)) = stack.pop() {
			let node = &self.nodes[node_index as usize];
			let cell_size = child_size(depth);
			for i in 0..SIZE_CUBED {
				let cell = node.contents[i as usize];
				if cell.kind() == CellKind::Empty {
					continue;
				}
				let child_origin = origin + (get_child_contents_pos(i).as_uvec3() * cell_size).as_ivec3();
				let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size as i32) };
				if !region.intersects(child_region) {
					continue;
				}
				match cell.kind() {
					CellKind::Empty => unreachable!(),
					CellKind::Data => f(child_origin, cell_size, cell.data_value()),
					CellKind::Node => stack.push((node_index + cell.node_offset(), depth - 1, child_origin)),
				}
			}
		}
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

					let cell = self.nodes[node_index as usize].get_child_cell_from_index(child_index);
					if cell.kind() == CellKind::Empty {
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
