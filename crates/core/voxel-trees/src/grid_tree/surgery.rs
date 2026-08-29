use smallvec::SmallVec;

use super::*;

impl<G: GridType> GridTree<G> {
	pub(super) fn occupied_count_in_cell(&self, node_depth: u8, node_index: u32, child_index: u8) -> u64 {
		match self.raw.cell_kind(node_index, child_index) {
			CellKind::Empty => 0,
			CellKind::Data => {
				let s = child_size(node_depth) as u64;
				s * s * s
			}
			CellKind::Node if node_depth == 0 => 1,
			CellKind::Node => self.occupied_count_in_node(self.raw.child_index(node_index, child_index), node_depth - 1),
		}
	}

	pub(super) fn occupied_count_in_node(&self, node_index: u32, node_depth: u8) -> u64 {
		(0..SIZE_CUBED).map(|child| self.occupied_count_in_cell(node_depth, node_index, child)).sum()
	}

	pub(super) fn mark_subtree_dead(&mut self, node_index: u32) {
		for i in 0..SIZE_CUBED {
			if self.raw.cell_kind(node_index, i) == CellKind::Node {
				self.mark_subtree_dead(self.raw.child_index(node_index, i));
			}
		}
		self.remove_node(node_index);
	}

	pub(super) fn set_child_area_to_data(&mut self, node_index: u32, node_depth: u8, child_index: u8, data: G::Data<'_>) {
		let old_kind = self.raw.cell_kind(node_index, child_index);
		let old_count = self.occupied_count_in_cell(node_depth, node_index, child_index);
		if old_kind == CellKind::Node {
			self.mark_subtree_dead(self.raw.child_index(node_index, child_index));
		}

		let s = child_size(node_depth) as u64;
		let new_count = s * s * s;
		if old_kind == CellKind::Empty {
			self.raw.inc_used_cell_count(node_index);
		}
		self.raw.set_data(&self.grid_type, node_index, child_index, data);
		if new_count >= old_count { self.raw.add_item_count(new_count - old_count); }
		else { self.raw.sub_item_count(old_count - new_count); }
	}

	pub(super) fn set_child_area_to_empty(&mut self, node_index: u32, node_depth: u8, child_index: u8) {
		let old_kind = self.raw.cell_kind(node_index, child_index);
		if old_kind == CellKind::Empty {
			return;
		}
		let old_count = self.occupied_count_in_cell(node_depth, node_index, child_index);
		if old_kind == CellKind::Node {
			self.mark_subtree_dead(self.raw.child_index(node_index, child_index));
		}

		self.raw.dec_used_cell_count(node_index);
		self.raw.set_empty(node_index, child_index);
		self.raw.sub_item_count(old_count);
	}

	pub(super) fn set_voxel_child_to_data(&mut self, node_index: u32, child_index: u8, data: G::Data<'_>) {
		let old_kind = self.raw.cell_kind(node_index, child_index);
		match old_kind {
			CellKind::Empty => {
				self.raw.inc_used_cell_count(node_index);
				self.raw.add_item_count(1);
			}
			CellKind::Data => {}
			CellKind::Node => self.mark_subtree_dead(self.raw.child_index(node_index, child_index)),
		}
		self.raw.set_data(&self.grid_type, node_index, child_index, data);
	}

	pub(super) fn allocate_empty_child_node(&mut self, parent_index: u32, child_index: u8) -> Option<u32> {
		let child_index_u32 = self.alloc_empty_node_after_parent(parent_index)?;
		let old = self.raw.cell_kind(parent_index, child_index);
		if old == CellKind::Empty {
			self.raw.inc_used_cell_count(parent_index);
		}
		self.raw.set_child_index(parent_index, child_index, child_index_u32);
		Some(child_index_u32)
	}

	pub(super) fn allocate_data_child_node(&mut self, parent_index: u32, child_index: u8, data: G::Data<'_>) -> Option<u32> {
		let child_index_u32 = self.alloc_node_after_parent_with_data(parent_index, data)?;
		let old = self.raw.cell_kind(parent_index, child_index);
		if old == CellKind::Empty {
			self.raw.inc_used_cell_count(parent_index);
		}
		self.raw.set_child_index(parent_index, child_index, child_index_u32);
		Some(child_index_u32)
	}

	pub(super) fn allocate_child_node_copying_cell(&mut self, parent_index: u32, child_index: u8) -> Option<u32> {
		let child_index_u32 = self.alloc_node_after_parent_copying_cell(parent_index, parent_index, child_index)?;
		let old = self.raw.cell_kind(parent_index, child_index);
		if old == CellKind::Empty {
			self.raw.inc_used_cell_count(parent_index);
		}
		self.raw.set_child_index(parent_index, child_index, child_index_u32);
		Some(child_index_u32)
	}

	pub(super) fn collapse_child_node_if_possible(&mut self, parent_index: u32, child_index: u8) {
		if self.raw.cell_kind(parent_index, child_index) != CellKind::Node {
			return;
		}
		let child_index_abs = self.raw.child_index(parent_index, child_index);
		let used = self.raw.used_cell_count(child_index_abs);
		if used == 0 {
			self.remove_node(child_index_abs);
			self.raw.dec_used_cell_count(parent_index);
			self.raw.set_empty(parent_index, child_index);
			return;
		}
		if used != SIZE_CUBED || self.raw.cell_kind(child_index_abs, 0) != CellKind::Data {
			return;
		}
		for cell in 1..SIZE_CUBED {
			if self.raw.cell_kind(child_index_abs, cell) != CellKind::Data || !self.data_cells_eq(child_index_abs, 0, child_index_abs, cell) {
				return;
			}
		}
		self.raw.copy_data_cell(child_index_abs, 0, parent_index, child_index);
		self.remove_node(child_index_abs);
	}

	pub(super) fn add_areas_recurse<'a>(&mut self, node_index: u32, node_depth: u8, node_origin: UVec3, ops: &[AreaOp<'a, G>]) -> bool {
		if ops.is_empty() {
			return true;
		}

		let cell_size = child_size(node_depth);
		let mut child_ops: [SmallVec<[AreaOp<'a, G>; 10]>; SIZE_USIZE_CUBED] = std::array::from_fn(|_| SmallVec::default());

		let node_end = node_origin + UVec3::splat(size(node_depth));
		for op in ops {
			let overlap_min = op.region.min().as_uvec3().max(node_origin);
			let overlap_end = op.region.end().as_uvec3().min(node_end);
			if overlap_min.cmpge(overlap_end).any() {
				continue;
			}
			let child_min = (overlap_min - node_origin) / UVec3::splat(cell_size);
			let child_max = (overlap_end - node_origin - UVec3::ONE) / UVec3::splat(cell_size);
			for z in child_min.z..=child_max.z {
				for y in child_min.y..=child_max.y {
					for x in child_min.x..=child_max.x {
						let i = (x + y * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as usize;
						child_ops[i].push(*op);
					}
				}
			}
		}

		if node_depth == 0 {
			for i in 0..SIZE_CUBED {
				let Some(last) = child_ops[i as usize].last() else { continue };
				self.set_voxel_child_to_data(node_index, i, last.data);
			}
			return true;
		}

		for i in 0..SIZE_CUBED {
			let bucket = &child_ops[i as usize];
			if bucket.is_empty() {
				continue;
			}

			let child_origin = node_origin + get_child_contents_pos(i).as_uvec3() * cell_size;
			let child_end = child_origin + UVec3::splat(cell_size);
			let mut partial_run_start = 0usize;

			for op_index in 0..bucket.len() {
				let op = bucket[op_index];
				let fully_covered = op.region.min().as_uvec3().cmple(child_origin).all() && child_end.cmple(op.region.end().as_uvec3()).all();
				if !fully_covered && node_depth != 0 {
					continue;
				}

				if partial_run_start < op_index {
					let child_node_index = match self.child_node_for_partial_area(node_index, i) {
						Some(index) => index,
						None => return false,
					};
					if !self.add_areas_recurse(child_node_index, node_depth - 1, child_origin, &bucket[partial_run_start..op_index]) {
						return false;
					}
					self.collapse_child_node_if_possible(node_index, i);
				}

				self.set_child_area_to_data(node_index, node_depth, i, op.data);
				partial_run_start = op_index + 1;
			}

			if partial_run_start < bucket.len() {
				let child_node_index = match self.child_node_for_partial_area(node_index, i) {
					Some(index) => index,
					None => return false,
				};
				if !self.add_areas_recurse(child_node_index, node_depth - 1, child_origin, &bucket[partial_run_start..]) {
					return false;
				}
				self.collapse_child_node_if_possible(node_index, i);
			}
		}
		true
	}

	pub(super) fn child_node_for_partial_area(&mut self, node_index: u32, child_index: u8) -> Option<u32> {
		match self.raw.cell_kind(node_index, child_index) {
			CellKind::Node => Some(self.raw.child_index(node_index, child_index)),
			CellKind::Empty => self.allocate_empty_child_node(node_index, child_index),
			CellKind::Data => self.allocate_child_node_copying_cell(node_index, child_index),
		}
	}
}
