use super::*;

impl<C: GridCell, Co: GridCoord> GridTree<C, Co> {
	pub(super) fn occupied_count_in_cell(&self, node_index: u32, node_depth: u8, cell: C) -> u64 {
		match cell.kind() {
			CellKind::Empty => 0,
			CellKind::Data => {
				let s = child_size(node_depth) as u64;
				s * s * s
			}
			// A node at depth 0 cannot represent more than this single voxel-sized
			// cell. Treat it as occupied if encountered while replacing legacy or
			// partially-collapsed structure instead of underflowing the depth.
			CellKind::Node if node_depth == 0 => 1,
			CellKind::Node => self.occupied_count_in_node(node_index + cell.node_offset(), node_depth - 1),
		}
	}

	pub(super) fn occupied_count_in_node(&self, node_index: u32, node_depth: u8) -> u64 {
		self.nodes[node_index as usize].contents.iter().map(|cell| self.occupied_count_in_cell(node_index, node_depth, *cell)).sum()
	}

	pub(super) fn mark_subtree_dead(&mut self, node_index: u32) {
		for i in 0..SIZE_CUBED {
			let cell = self.nodes[node_index as usize].contents[i as usize];
			if cell.kind() == CellKind::Node {
				self.mark_subtree_dead(node_index + cell.node_offset());
			}
		}
		self.remove_node(node_index);
	}

	pub(super) fn set_child_area_to_data(&mut self, node_index: u32, node_depth: u8, child_index: u8, data: C::Data) {
		let old = self.nodes[node_index as usize].get_child_cell_from_index(child_index);
		let old_count = self.occupied_count_in_cell(node_index, node_depth, old);
		if old.kind() == CellKind::Node {
			self.mark_subtree_dead(node_index + old.node_offset());
		}

		let s = child_size(node_depth) as u64;
		let new_count = s * s * s;
		let node = &mut self.nodes[node_index as usize];
		if old.kind() == CellKind::Empty {
			node.used_cell_count += 1;
			assert!(node.used_cell_count <= 64);
		}
		node.set_child_cell_from_index(child_index, C::data(data));
		self.item_count = self.item_count + new_count - old_count;
	}

	pub(super) fn set_child_area_to_empty(&mut self, node_index: u32, node_depth: u8, child_index: u8) {
		let old = self.nodes[node_index as usize].get_child_cell_from_index(child_index);
		if old.kind() == CellKind::Empty {
			return;
		}
		let old_count = self.occupied_count_in_cell(node_index, node_depth, old);
		if old.kind() == CellKind::Node {
			self.mark_subtree_dead(node_index + old.node_offset());
		}

		let node = &mut self.nodes[node_index as usize];
		node.used_cell_count -= 1;
		node.set_child_cell_to_none_from_index(child_index);
		self.item_count -= old_count;
	}

	pub(super) fn set_voxel_child_to_data(&mut self, node_index: u32, child_index: u8, data: C::Data) {
		let old = self.nodes[node_index as usize].get_child_cell_from_index(child_index);
		let node = &mut self.nodes[node_index as usize];
		match old.kind() {
			CellKind::Empty => {
				node.used_cell_count += 1;
				assert!(node.used_cell_count <= 64);
				self.item_count += 1;
			}
			CellKind::Data => {}
			CellKind::Node => {
				// A depth-0 child should not normally be a node, but preserve the
				// replacement semantics if older/corrupt structure is encountered.
				self.mark_subtree_dead(node_index + old.node_offset());
			}
		}
		self.nodes[node_index as usize].set_child_cell_from_index(child_index, C::data(data));
	}

	pub(super) fn allocate_child_node(&mut self, parent_index: u32, child_index: u8, contents: C) -> Option<u32> {
		if self.nodes.len() + MAX_TREE_DEPTH_USIZE > C::MAX_NODE_OFFSET as usize {
			bevy::log::warn!("GridTree node arena full ({} live); skipping partial add_area edit", self.nodes.len());
			return None;
		}
		let child_index_u32 = self.nodes.len() as u32;
		let offset = child_index_u32 - parent_index;
		if offset == 0 || offset > C::MAX_NODE_OFFSET {
			bevy::log::warn!("GridTree node offset overflow; skipping partial add_area edit");
			return None;
		}
		let used_cell_count = if contents.kind() == CellKind::Empty { 0 } else { SIZE_CUBED };
		let old = self.nodes[parent_index as usize].get_child_cell_from_index(child_index);
		let parent = &mut self.nodes[parent_index as usize];
		if old.kind() == CellKind::Empty {
			parent.used_cell_count += 1;
			assert!(parent.used_cell_count <= 64);
		}
		parent.set_child_cell_to_node_from_index(child_index, offset);
		self.nodes.push(GridTreeNode { contents: [contents; SIZE_USIZE_CUBED], parent_offset: offset as u16, used_cell_count });
		Some(child_index_u32)
	}

	pub(super) fn collapse_child_node_if_possible(&mut self, parent_index: u32, child_index: u8) {
		let cell = self.nodes[parent_index as usize].get_child_cell_from_index(child_index);
		if cell.kind() != CellKind::Node {
			return;
		}
		let child_index_abs = parent_index + cell.node_offset();
		let child = &self.nodes[child_index_abs as usize];
		if child.used_cell_count == 0 {
			self.remove_node(child_index_abs);
			let parent = &mut self.nodes[parent_index as usize];
			parent.used_cell_count -= 1;
			parent.set_child_cell_to_none_from_index(child_index);
			return;
		}
		if child.used_cell_count != SIZE_CUBED {
			return;
		}
		let first = child.contents[0];
		if first.kind() != CellKind::Data || child.contents.iter().any(|cell| *cell != first) {
			return;
		}
		let data = first.data_value();
		self.remove_node(child_index_abs);
		self.nodes[parent_index as usize].set_child_cell_from_index(child_index, C::data(data));
	}

	pub(super) fn add_areas_recurse(&mut self, node_index: u32, node_depth: u8, node_origin: IVec3, ops: &[AreaOp<C::Data>]) -> bool {
		if ops.is_empty() {
			return true;
		}

		let cell_size = child_size(node_depth);
		let mut child_ops: [Vec<AreaOp<C::Data>>; SIZE_USIZE_CUBED] = std::array::from_fn(|_| Vec::new());

		// Spatially bucket the whole batch by this node's children while preserving
		// input order inside each bucket, so overlapping writes retain sequential
		// semantics but traversal is shared by neighbouring/sibling edits. Compute
		// the intersected child index range directly instead of testing all 64
		// children for every op; dense single-voxel batches hit exactly one child per
		// level.
		let node_end = node_origin + IVec3::splat(size(node_depth) as i32);
		let cell_size_i = cell_size as i32;
		for op in ops {
			let overlap_min = op.min.max(node_origin);
			let overlap_end = op.end.min(node_end);
			if overlap_min.cmpge(overlap_end).any() {
				continue;
			}
			let child_min = (overlap_min - node_origin).div_euclid(IVec3::splat(cell_size_i));
			let child_max = (overlap_end - node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size_i));
			for z in child_min.z..=child_max.z {
				for y in child_min.y..=child_max.y {
					for x in child_min.x..=child_max.x {
						let i = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as usize;
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

			let child_origin = node_origin + (get_child_contents_pos(i).as_uvec3() * cell_size).as_ivec3();
			let child_end = child_origin + IVec3::splat(cell_size as i32);
			let mut partial_run_start = 0usize;

			for op_index in 0..bucket.len() {
				let op = bucket[op_index];
				let fully_covered = op.min.cmple(child_origin).all() && child_end.cmple(op.end).all();
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
				if node_depth == 0 {
					for op in &bucket[partial_run_start..] {
						self.set_child_area_to_data(node_index, node_depth, i, op.data);
					}
					continue;
				}
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
		let cell = self.nodes[node_index as usize].get_child_cell_from_index(child_index);
		match cell.kind() {
			CellKind::Node => Some(node_index + cell.node_offset()),
			CellKind::Empty => self.allocate_child_node(node_index, child_index, C::EMPTY),
			CellKind::Data => self.allocate_child_node(node_index, child_index, cell),
		}
	}

}
