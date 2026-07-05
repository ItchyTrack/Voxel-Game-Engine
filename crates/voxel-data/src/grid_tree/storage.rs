use super::*;

impl<C: GridCell, Co: GridCoord> GridTree<C, Co> {
	#[inline]
	pub(super) fn max_extent() -> i32 {
		size(Co::MAX_ROOT_DEPTH) as i32
	}

	#[inline]
	pub(super) fn canonical_extent_region() -> GridRegion {
		GridRegion { min: IVec3::ZERO, end: IVec3::splat(Self::max_extent()) }
	}

	#[inline]
	pub(super) fn reset_empty_root(&mut self, root_pos: IVec3, root_depth: u8) {
		self.nodes.clear();
		self.nodes.push(GridTreeNode::new_root());
		self.root_pos = root_pos;
		self.root_depth = root_depth;
		self.item_count = 0;
		self.dead_nodes = 0;
		self.free_nodes.clear();
	}

	#[inline]
	pub(super) fn canonical_root_for_bounds(min: IVec3, max: IVec3) -> Option<(IVec3, u8)> {
		if min.cmplt(IVec3::ZERO).any() || max.cmplt(min).any() || max.cmpge(IVec3::splat(Self::max_extent())).any() {
			return None;
		}
		for depth in 0..=Co::MAX_ROOT_DEPTH {
			let cube = size(depth) as i32;
			let origin = min.div_euclid(IVec3::splat(cube)) * cube;
			let end = origin + IVec3::splat(cube);
			if max.cmplt(end).all() {
				return Some((origin, depth));
			}
		}
		None
	}

	pub(super) fn make_sure_root_covers_pos(&mut self, pos: IVec3) -> bool {
		if self.nodes[0].used_cell_count == 0 {
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

	pub(super) fn root_covers(&self, pos: IVec3) -> bool {
		let r = pos - self.root_pos;
		r.min_element() >= 0 && (r.max_element() as i64) < size(self.root_depth) as i64
	}

	pub(super) fn make_sure_root_covers_area(&mut self, min: IVec3, max: IVec3) -> bool {
		if self.nodes[0].used_cell_count == 0 {
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

	pub(super) fn promote_root_to_cover_bounds(&mut self, min: IVec3, max: IVec3) -> bool {
		let existing_max = self.root_pos + IVec3::splat(size(self.root_depth) as i32 - 1);
		let cover_min = self.root_pos.min(min);
		let cover_max = existing_max.max(max);
		let Some((target_pos, target_depth)) = Self::canonical_root_for_bounds(cover_min, cover_max) else {
			bevy::log::warn!("GridTree bounds {min:?}..={max:?} are outside canonical positive extent {:?}; skipping edit", Self::canonical_extent_region());
			return false;
		};
		if self.nodes[0].used_cell_count == 0 {
			self.reset_empty_root(target_pos, target_depth);
			return true;
		}
		while self.root_depth < target_depth || self.root_pos != target_pos {
			let new_depth = self.root_depth + 1;
			let new_size = size(new_depth) as i32;
			let new_pos = self.root_pos.div_euclid(IVec3::splat(new_size)) * new_size;
			let rel = (self.root_pos - new_pos).div_euclid(IVec3::splat(size(self.root_depth) as i32));
			let child_index = get_child_contents_index(rel.as_u8vec3());
			self.nodes.insert(0, GridTreeNode::new_root());
			self.nodes[1].parent_offset = 1;
			self.nodes[0].used_cell_count = 1;
			self.nodes[0].set_child_cell_to_node_from_index(child_index, 1);
			// Descendant parent offsets stay unchanged: inserting a new root at index 0
			// shifts both a node and its parent by +1, so their relative distance is the same.
			// Only pre-existing nodes need their child pointers incremented; the freshly
			// inserted root already points at the shifted old root index (1).
			for node in self.nodes.iter_mut().skip(1) {
				for cell in &mut node.contents {
					if cell.kind() == CellKind::Node {
						*cell = C::node(cell.node_index() + 1);
					}
				}
			}
			for free in &mut self.free_nodes {
				*free += 1;
			}
			self.root_pos = new_pos;
			self.root_depth = new_depth;
		}
		self.root_covers(min) && self.root_covers(max)
	}


	pub(super) fn alloc_node_after_parent(&mut self, parent_node_index: u32, contents: C, used_cell_count: u8) -> Option<u32> {
		if let Some(free_slot) = self.free_nodes.iter().position(|&idx| idx > parent_node_index && idx - parent_node_index <= C::MAX_NODE_OFFSET) {
			let node_index = self.free_nodes.swap_remove(free_slot);
			self.dead_nodes -= 1;
			self.nodes[node_index as usize] = GridTreeNode {
				contents: [contents; SIZE_USIZE_CUBED],
				parent_offset: (node_index - parent_node_index) as u16,
				used_cell_count,
			};
			return Some(node_index);
		}
		let node_index = self.nodes.len() as u32;
		let offset = node_index.checked_sub(parent_node_index)?;
		if offset == 0 || offset > C::MAX_NODE_OFFSET {
			return None;
		}
		self.nodes.push(GridTreeNode { contents: [contents; SIZE_USIZE_CUBED], parent_offset: offset as u16, used_cell_count });
		Some(node_index)
	}

	/// Expand a uniform data cell into a child node, then write one voxel within it.
	pub(super) fn set_voxel_in_data_cell(&mut self, parent_node_index: u32, parent_depth: u8, current_cell: C, cell_to_set: C, pos: UVec3) {
		let child_size = child_size(parent_depth);
		let relative_pos = (pos / child_size).as_u8vec3();
		let Some(child_index) = self.alloc_node_after_parent(parent_node_index, current_cell, SIZE_CUBED) else { return; };
		let parent = &mut self.nodes[parent_node_index as usize];
		parent.set_child_cell_to_node(relative_pos, child_index);
		if parent_depth == 1 {
			let node = &mut self.nodes[child_index as usize];
			if cell_to_set.kind() == CellKind::Empty {
				node.used_cell_count -= 1;
			}
			node.set_child_cell((pos % SIZE as u32).as_u8vec3(), cell_to_set);
		} else {
			self.set_voxel_in_data_cell(child_index, parent_depth - 1, current_cell, cell_to_set, pos % child_size);
		}
	}
	/// Expand an empty cell into a child node, then write one voxel within it.
	pub(super) fn set_voxel_in_none_cell(&mut self, parent_node_index: u32, parent_depth: u8, cell_to_set: C, pos: UVec3) {
		assert!(cell_to_set.kind() == CellKind::Data);
		let child_size = child_size(parent_depth);
		let contents_pos = (pos / child_size).as_u8vec3();
		let Some(child_index) = self.alloc_node_after_parent(parent_node_index, C::EMPTY, 0) else { return; };
		let parent = &mut self.nodes[parent_node_index as usize];
		parent.used_cell_count += 1;
		assert!(parent.used_cell_count <= 64);
		parent.set_child_cell_to_node(contents_pos, child_index);
		if parent_depth == 1 {
			let node = &mut self.nodes[child_index as usize];
			node.used_cell_count += 1;
			node.set_child_cell((pos % SIZE as u32).as_u8vec3(), cell_to_set);
		} else {
			self.set_voxel_in_none_cell(child_index, parent_depth - 1, cell_to_set, pos % child_size);
		}
	}
	pub fn internals(&self) -> (&Vec<GridTreeNode<C>>, Co::Pos, u8) {
		(&self.nodes, Co::from_ivec3(self.root_pos), self.root_depth)
	}
	/// Collapse a full uniform child subtree back into one data cell and continue upward.
	pub(super) fn try_merge(&mut self, node_index: u32, data: C::Data, cell_index_stack: &[u8]) {
		let node = &mut self.nodes[node_index as usize];
		if let Some(parent_offset) = node.get_parent_offset() {
			if node.used_cell_count != SIZE_CUBED {
				return;
			}
			for cell_index in 0..SIZE_CUBED {
				let cell = node.contents[cell_index as usize];
				if cell != C::data(data) {
					return;
				}
			}
			self.remove_node(node_index);
			let parent_index = node_index - parent_offset as u32;
			self.nodes[parent_index as usize].contents[cell_index_stack[cell_index_stack.len() - 1] as usize] = C::data(data);
			self.try_merge(parent_index, data, &cell_index_stack[0..(cell_index_stack.len() - 1)]);
		}
	}
	pub(super) fn try_merge_empty(&mut self, node_index: u32, cell_index_stack: &[u8]) {
		let node = &mut self.nodes[node_index as usize];
		if let Some(parent_offset) = node.get_parent_offset() {
			if node.used_cell_count != 0 {
				return;
			}
			self.remove_node(node_index);
			let parent_index = node_index - parent_offset as u32;
			let parent_node = &mut self.nodes[parent_index as usize];
			parent_node.used_cell_count -= 1;
			parent_node.set_child_cell_to_none_from_index(cell_index_stack[cell_index_stack.len() - 1]);
			self.try_merge_empty(parent_index, &cell_index_stack[0..(cell_index_stack.len() - 1)]);
		}
	}
	/// Mark a node slot as free for reuse.
	pub fn remove_node(&mut self, node_index: u32) {
		if let Some(node) = self.nodes.get_mut(node_index as usize) {
			if node.used_cell_count == 255 {
				return;
			}
			node.used_cell_count = 255; // mark as deleted
			self.dead_nodes += 1;
			self.free_nodes.push(node_index);
		}
	}


	/// True if there is room to allocate up to `MAX_TREE_DEPTH` additional nodes.
	/// Free-list reuse is the primary allocation strategy.
	pub(super) fn has_node_budget(&mut self) -> bool {
		if self.free_nodes.len() >= MAX_TREE_DEPTH_USIZE || self.nodes.len() + MAX_TREE_DEPTH_USIZE <= C::MAX_NODE_OFFSET as usize {
			return true;
		}
		bevy::log::warn!("GridTree node arena full ({} total, {} free); skipping edit", self.nodes.len(), self.free_nodes.len());
		false
	}
}
