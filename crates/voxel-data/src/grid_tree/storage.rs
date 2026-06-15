use super::*;

impl<C: GridCell, Co: GridCoord> GridTree<C, Co> {
	pub(super) fn make_sure_root_covers_pos(&mut self, pos: IVec3) -> bool {
		if self.nodes[0].used_cell_count == 0 {
			self.nodes = vec![GridTreeNode::new_root()];
			self.root_pos = pos;
			self.root_depth = 0;
			self.item_count = 0;
			return true;
		}
		if self.root_covers(pos) {
			return true;
		}
		self.rebuild_to_cover(pos)
	}

	pub(super) fn root_covers(&self, pos: IVec3) -> bool {
		let r = pos - self.root_pos;
		r.min_element() >= 0 && (r.max_element() as i64) < size(self.root_depth) as i64
	}

	pub(super) fn make_sure_root_covers_area(&mut self, min: IVec3, max: IVec3) -> bool {
		if self.nodes[0].used_cell_count == 0 {
			let span = (max - min).max_element() as i64 + 1;
			let mut depth = 0u8;
			while (size(depth) as i64) < span {
				if depth >= Co::MAX_ROOT_DEPTH {
					bevy::log::warn!("GridTree can't cover span {span} for area {min:?}..={max:?}; skipping add_area");
					return false;
				}
				depth += 1;
			}
			self.nodes = vec![GridTreeNode::new_root()];
			self.root_pos = min;
			self.root_depth = depth;
			self.item_count = 0;
			self.dead_nodes = 0;
			return true;
		}
		if self.root_covers(min) && self.root_covers(max) {
			return true;
		}
		self.rebuild_to_cover_bounds(min, max)
	}

	pub(super) fn rebuild_to_cover(&mut self, pos: IVec3) -> bool {
		self.rebuild_to_cover_bounds(pos, pos)
	}

	pub(super) fn rebuild_to_cover_bounds(&mut self, mut min: IVec3, mut max: IVec3) -> bool {
		let mut areas: Vec<AreaOp<C::Data>> = Vec::new();
		self.each_leaf(|origin, cell_size, value| {
			let end = origin + IVec3::splat(cell_size as i32);
			min = min.min(origin);
			max = max.max(end - IVec3::ONE);
			areas.push(AreaOp { min: origin, end, data: value });
		});
		let span = (max - min).max_element() as i64 + 1;
		let mut depth = 0u8;
		while (size(depth) as i64) < span {
			if depth >= Co::MAX_ROOT_DEPTH {
				bevy::log::warn!("GridTree can't cover span {span} for bounds {min:?}..={max:?}; skipping edit");
				return false;
			}
			depth += 1;
		}
		self.nodes = vec![GridTreeNode::new_root()];
		self.root_pos = min;
		self.root_depth = depth;
		self.item_count = 0;
		self.dead_nodes = 0;

		for attempt in 0..3 {
			if self.add_areas_recurse(0, self.root_depth, self.root_pos, &areas) {
				return true;
			}
			if attempt < 2 {
				self.compact();
			}
		}
		bevy::log::warn!("GridTree could not finish rebuild_to_cover_bounds after compaction retries");
		false
	}

	/// Visit every DATA leaf as (world origin, cell size, value) via an internal DFS.
	pub(super) fn each_leaf(&self, mut f: impl FnMut(IVec3, u32, C::Data)) {
		if self.nodes.is_empty() {
			return;
		}
		let mut stack: Vec<(u32, u8, IVec3)> = vec![(0, self.root_depth, self.root_pos)];
		while let Some((node_index, depth, origin)) = stack.pop() {
			let node = &self.nodes[node_index as usize];
			let cell_size = child_size(depth);
			for i in 0..SIZE_CUBED {
				let cell = node.contents[i as usize];
				let child_origin = origin + (get_child_contents_pos(i).as_uvec3() * cell_size).as_ivec3();
				match cell.kind() {
					CellKind::Empty => {}
					CellKind::Data => f(child_origin, cell_size, cell.data_value()),
					CellKind::Node => stack.push((node_index + cell.node_offset(), depth - 1, child_origin)),
				}
			}
		}
	}

	/// parent depth must be more than 0 and at pos in parent there must be a data cell containing current_cell and current_cell != cell_to_set
	pub(super) fn set_voxel_in_data_cell(&mut self, parent_node_index: u32, parent_depth: u8, current_cell: C, cell_to_set: C, pos: UVec3) {
		debug_assert!((self.nodes.len() as u32 - parent_node_index) as usize <= C::MAX_NODE_OFFSET as usize);
		let next_node_offset = (self.nodes.len() as u32 - parent_node_index) as u16;
		let parent = &mut self.nodes[parent_node_index as usize];
		let child_size = child_size(parent_depth);
		let relative_pos = (pos / child_size).as_u8vec3();
		assert!(next_node_offset != 0);
		parent.set_child_cell_to_node(relative_pos, next_node_offset as u32);
		self.nodes.push(GridTreeNode { contents: [current_cell; SIZE_USIZE_CUBED], parent_offset: next_node_offset, used_cell_count: SIZE_CUBED });
		if parent_depth == 1 {
			let node = &mut self.nodes[(parent_node_index + next_node_offset as u32) as usize];
			if cell_to_set.kind() == CellKind::Empty {
				node.used_cell_count -= 1;
			}
			node.set_child_cell((pos % SIZE as u32).as_u8vec3(), cell_to_set);
		} else {
			self.set_voxel_in_data_cell(parent_node_index + next_node_offset as u32, parent_depth - 1, current_cell, cell_to_set, pos % child_size);
		}
	}
	/// parent depth must be more than 0 and at pos in parent there must be a none cell and cell_to_set must be DATA
	pub(super) fn set_voxel_in_none_cell(&mut self, parent_node_index: u32, parent_depth: u8, cell_to_set: C, pos: UVec3) {
		assert!(cell_to_set.kind() == CellKind::Data);
		debug_assert!((self.nodes.len() as u32 - parent_node_index) as usize <= C::MAX_NODE_OFFSET as usize);
		let next_node_offset = (self.nodes.len() as u32 - parent_node_index) as u16;
		let parent = &mut self.nodes[parent_node_index as usize];
		parent.used_cell_count += 1;
		assert!(parent.used_cell_count <= 64);
		let child_size = child_size(parent_depth);
		let contents_pos = (pos / child_size).as_u8vec3();
		assert!(next_node_offset != 0);
		parent.set_child_cell_to_node(contents_pos, next_node_offset as u32);
		self.nodes.push(GridTreeNode { contents: [C::EMPTY; SIZE_USIZE_CUBED], parent_offset: next_node_offset, used_cell_count: 0 });
		if parent_depth == 1 {
			let node = &mut self.nodes[(parent_node_index + next_node_offset as u32) as usize];
			node.used_cell_count += 1;
			node.set_child_cell((pos % SIZE as u32).as_u8vec3(), cell_to_set);
		} else {
			self.set_voxel_in_none_cell(parent_node_index + next_node_offset as u32, parent_depth - 1, cell_to_set, pos % child_size);
		}
	}
	pub fn internals(&self) -> (&Vec<GridTreeNode<C>>, Co::Pos, u8) {
		(&self.nodes, Co::from_ivec3(self.root_pos), self.root_depth)
	}
	/// cell_to_merge cant be NODE. pos_in_node is any pos
	pub(super) fn try_merge(&mut self, node_index: u32, data: C::Data, cell_index_stack: &[u8]) {
		let node = &mut self.nodes[node_index as usize];
		if let Some(parent_offset) = node.get_parent_offset() {
			// if it dont have a parent it cant be merged
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
			// if it dont have a parent it cant be merged
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
	/// Assumes childern are dead.
	pub fn remove_node(&mut self, node_index: u32) {
		if let Some(node) = self.nodes.get_mut(node_index as usize) {
			node.used_cell_count = 255; // mark as deleted
			self.dead_nodes += 1;
		} else {
			println!("NODE GONE!");
		}
	}

	pub(super) fn maybe_compact(&mut self) {
		if self.dead_nodes * 2 > self.nodes.len() && self.nodes.len() > 64 {
			self.compact();
		}
	}

	/// Rewrite the node arena in DFS order, dropping dead nodes and re-deriving
	/// every child offset. Preserves the logical tree (root_pos/depth, item_count
	/// and all data are unchanged) — only node indices/offsets change.
	pub(super) fn compact(&mut self) {
		let live = self.nodes.len() - self.dead_nodes;
		let mut new_nodes: Vec<GridTreeNode<C>> = Vec::with_capacity(live);
		new_nodes.push(self.nodes[0].clone()); // root keeps parent_offset 0
		let mut stack: Vec<(u32, u32)> = vec![(0, 0)]; // (old_index, new_index)
		while let Some((old_idx, new_idx)) = stack.pop() {
			for ci in 0..SIZE_CUBED {
				let cell = self.nodes[old_idx as usize].contents[ci as usize];
				if cell.kind() == CellKind::Node {
					let child_old = old_idx + cell.node_offset();
					let child_new = new_nodes.len() as u32;
					let mut child = self.nodes[child_old as usize].clone();
					child.parent_offset = (child_new - new_idx) as u16;
					new_nodes.push(child);
					new_nodes[new_idx as usize].set_child_cell_to_node_from_index(ci, child_new - new_idx);
					stack.push((child_old, child_new));
				}
			}
		}
		self.nodes = new_nodes;
		self.dead_nodes = 0;
	}

	/// True if there is room to allocate up to `MAX_TREE_DEPTH` new nodes without
	/// overflowing the 15-bit node offset. Compacts first; the arena (after
	/// compaction) is the live node count.
	pub(super) fn has_node_budget(&mut self) -> bool {
		self.maybe_compact();
		if self.nodes.len() + MAX_TREE_DEPTH_USIZE <= C::MAX_NODE_OFFSET as usize {
			return true;
		}
		bevy::log::warn!("GridTree node arena full ({} live); skipping edit", self.nodes.len());
		false
	}
}
