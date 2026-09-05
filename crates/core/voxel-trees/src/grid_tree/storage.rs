use super::*;

impl<G: GridType> GridTree<G> {
	#[inline]
	pub(super) fn max_extent() -> u32 {
		size(13) // may be 6 for voxels... need to add this param somewhere else
	}

	#[inline]
	pub(super) fn canonical_extent_region() -> NonZeroVoxelRegion {
		NonZeroVoxelRegion::from_min_end(IVec3::ZERO, IVec3::splat(Self::max_extent() as i32)).unwrap()
	}

	#[inline]
	pub(super) fn reset_empty_root(&mut self, root_pos: UVec3, root_depth: u8) {
		self.raw.reset_empty_root(root_pos, root_depth);
	}

	#[inline]
	pub(super) fn canonical_root_for_bounds(min: UVec3, max: UVec3) -> Option<(UVec3, u8)> {
		if min.cmplt(UVec3::ZERO).any() || max.cmplt(min).any() || max.cmpge(UVec3::splat(Self::max_extent())).any() {
			return None;
		}
		for depth in 0..=13/*Co::MAX_ROOT_DEPTH*/ {
			let cube = size(depth);
			let origin = min / UVec3::splat(cube) * cube;
			let end = origin + UVec3::splat(cube);
			if max.cmplt(end).all() {
				return Some((origin, depth));
			}
		}
		None
	}

	pub(super) fn make_sure_root_covers_pos(&mut self, pos: UVec3) -> bool {
		if self.raw.used_cell_count(0) == 0 {
			let Some((root_pos, root_depth)) = Self::canonical_root_for_bounds(pos, pos) else {
				bevy::log::warn!("GridTree position {pos:?} is outside canonical positive extent {:?}; skipping insert", Self::canonical_extent_region());
				return false;
			};
			self.reset_empty_root(root_pos, root_depth);
			return true;
		}
		if self.root_covers(pos) {
			return true;
		}
		self.promote_root_to_cover_bounds(pos, pos)
	}

	pub(super) fn root_covers(&self, pos: UVec3) -> bool {
		let r = pos.as_ivec3() - self.raw.root_pos().as_ivec3();
		r.min_element() >= 0 && (r.max_element() as i64) < size(self.raw.root_depth()) as i64
	}

	pub(super) fn make_sure_root_covers_area(&mut self, min: UVec3, max: UVec3) -> bool {
		if self.raw.used_cell_count(0) == 0 {
			let Some((root_pos, root_depth)) = Self::canonical_root_for_bounds(min, max) else {
				bevy::log::warn!("GridTree area {min:?}..={max:?} is outside canonical positive extent {:?}; skipping edit", Self::canonical_extent_region());
				return false;
			};
			self.reset_empty_root(root_pos, root_depth);
			return true;
		}
		if self.root_covers(min) && self.root_covers(max) {
			return true;
		}
		self.promote_root_to_cover_bounds(min, max)
	}

	pub(super) fn promote_root_to_cover_bounds(&mut self, min: UVec3, max: UVec3) -> bool {
		let existing_max = self.raw.root_pos() + UVec3::splat(size(self.raw.root_depth()) - 1);
		let cover_min = self.raw.root_pos().min(min);
		let cover_max = existing_max.max(max);
		let Some((target_pos, target_depth)) = Self::canonical_root_for_bounds(cover_min, cover_max) else {
			bevy::log::warn!("GridTree bounds {min:?}..={max:?} are outside canonical positive extent {:?}; skipping edit", Self::canonical_extent_region());
			return false;
		};
		if self.raw.used_cell_count(0) == 0 {
			self.reset_empty_root(target_pos, target_depth);
			return true;
		}
		while self.raw.root_depth() < target_depth || self.raw.root_pos() != target_pos {
			let old_root_pos = self.raw.root_pos();
			let old_root_depth = self.raw.root_depth();
			let new_depth = old_root_depth + 1;
			let new_size = size(new_depth);
			let new_pos = old_root_pos / UVec3::splat(new_size) * new_size;
			let rel = (old_root_pos - new_pos) / UVec3::splat(size(old_root_depth));
			let child_index = get_child_contents_index(rel.as_u8vec3());
			self.raw.insert_empty_node(0, 0);
			self.raw.set_parent_offset(1, 1);
			self.raw.set_used_cell_count(0, 1);
			self.raw.set_child_index(0, child_index, 1);
			self.raw.increment_child_indices_from(1, 1);
			self.raw.set_root(new_pos, new_depth);
		}
		self.root_covers(min) && self.root_covers(max)
	}

	pub(super) fn alloc_empty_node_after_parent(&mut self, parent_node_index: u32) -> Option<u32> {
		let node_index = self.alloc_raw_node_after_parent(parent_node_index)?;
		self.raw.clear_node(node_index);
		self.raw.set_parent_offset(node_index, node_index - parent_node_index);
		Some(node_index)
	}

	pub(super) fn alloc_node_after_parent_with_data(&mut self, parent_node_index: u32, data: G::Data<'_>) -> Option<u32> {
		let node_index = self.alloc_empty_node_after_parent(parent_node_index)?;
		self.raw.fill_node_with_data(&self.grid_type, node_index, data);
		Some(node_index)
	}

	pub(super) fn alloc_node_after_parent_copying_cell(&mut self, parent_node_index: u32, src_node: u32, src_child: u8) -> Option<u32> {
		let node_index = self.alloc_empty_node_after_parent(parent_node_index)?;
		self.raw.copy_cell_to_all_children(src_node, src_child, node_index);
		Some(node_index)
	}

	fn alloc_raw_node_after_parent(&mut self, parent_node_index: u32) -> Option<u32> {
		if let Some(node_index) = self.raw.free_node_after_parent(parent_node_index, G::MAX_NODE_OFFSET) {
			return Some(node_index);
		}
		let node_index = self.raw.node_count() as u32;
		let offset = node_index.checked_sub(parent_node_index)?;
		if offset == 0 || offset > G::MAX_NODE_OFFSET {
			return None;
		}
		Some(self.raw.push_empty_node(offset))
	}

	/// Expand a uniform data cell into a child node, then write one voxel within it.
	pub(super) fn set_voxel_in_data_cell<'a>(&mut self, parent_node_index: u32, parent_depth: u8, cell_to_set: CellWrite<'a, G>, pos: bevy::math::UVec3) {
		let child_size = child_size(parent_depth);
		let parent_child_index = get_child_contents_index((pos / child_size).as_u8vec3());
		let Some(child_index) = self.alloc_node_after_parent_copying_cell(parent_node_index, parent_node_index, parent_child_index) else { return; };
		self.raw.set_child_index(parent_node_index, parent_child_index, child_index);
		if parent_depth == 1 {
			let voxel_child = get_child_contents_index((pos % SIZE as u32).as_u8vec3());
			match cell_to_set {
				CellWrite::Empty => {
					self.raw.set_empty(child_index, voxel_child);
					self.raw.dec_used_cell_count(child_index);
				}
				CellWrite::Data(data) => self.raw.set_data(&self.grid_type, child_index, voxel_child, data),
			}
		} else {
			self.set_voxel_in_data_cell(child_index, parent_depth - 1, cell_to_set, pos % child_size);
		}
	}

	/// Expand an empty cell into a child node, then write one voxel within it.
	pub(super) fn set_voxel_in_none_cell<'a>(&mut self, parent_node_index: u32, parent_depth: u8, cell_to_set: G::Data<'a>, pos: bevy::math::UVec3) {
		let child_size = child_size(parent_depth);
		let contents_pos = (pos / child_size).as_u8vec3();
		let contents_index = get_child_contents_index(contents_pos);
		let Some(child_index) = self.alloc_empty_node_after_parent(parent_node_index) else { return; };
		self.raw.inc_used_cell_count(parent_node_index);
		self.raw.set_child_index(parent_node_index, contents_index, child_index);
		if parent_depth == 1 {
			let voxel_child = get_child_contents_index((pos % SIZE as u32).as_u8vec3());
			self.raw.inc_used_cell_count(child_index);
			self.raw.set_data(&self.grid_type, child_index, voxel_child, cell_to_set);
		} else {
			self.set_voxel_in_none_cell(child_index, parent_depth - 1, cell_to_set, pos % child_size);
		}
	}

	pub fn internals(&self) -> (GridTreeViewImpl<'_, G>, UVec3, u8) {
		(self.view(), self.raw.root_pos(), self.raw.root_depth())
	}

	/// Collapse a full uniform child subtree back into one data cell and continue upward.
	pub(super) fn try_merge(&mut self, node_index: u32, data: G::Data<'_>, cell_index_stack: &[u8]) {
		let parent_offset = self.raw.parent_offset(node_index);
		if parent_offset == 0 || self.raw.used_cell_count(node_index) != SIZE_CUBED {
			return;
		}
		for cell_index in 0..SIZE_CUBED {
			if self.raw.cell_kind(node_index, cell_index) != CellKind::Data || !self.cell_data_eq(node_index, cell_index, data) {
				return;
			}
		}
		self.remove_node(node_index);
		let parent_index = node_index - parent_offset;
		self.raw.set_data(&self.grid_type, parent_index, cell_index_stack[cell_index_stack.len() - 1], data);
		self.try_merge(parent_index, data, &cell_index_stack[0..(cell_index_stack.len() - 1)]);
	}

	pub(super) fn try_merge_empty(&mut self, node_index: u32, cell_index_stack: &[u8]) {
		let parent_offset = self.raw.parent_offset(node_index);
		if parent_offset == 0 || self.raw.used_cell_count(node_index) != 0 {
			return;
		}
		self.remove_node(node_index);
		let parent_index = node_index - parent_offset;
		self.raw.dec_used_cell_count(parent_index);
		self.raw.set_empty(parent_index, cell_index_stack[cell_index_stack.len() - 1]);
		self.try_merge_empty(parent_index, &cell_index_stack[0..(cell_index_stack.len() - 1)]);
	}

	/// Mark a node slot as free for reuse.
	pub fn remove_node(&mut self, node_index: u32) {
		self.raw.remove_node(node_index);
	}

	/// True if there is room to allocate up to `MAX_TREE_DEPTH` additional nodes.
	/// Free-list reuse is the primary allocation strategy.
	pub(super) fn has_node_budget(&mut self) -> bool {
		if self.raw.free_node_count() >= MAX_TREE_DEPTH_USIZE || self.raw.node_count() + MAX_TREE_DEPTH_USIZE <= G::MAX_NODE_OFFSET as usize {
			return true;
		}
		bevy::log::warn!("GridTree node arena full ({} total, {} free); skipping edit", self.raw.node_count(), self.raw.free_node_count());
		false
	}
}
