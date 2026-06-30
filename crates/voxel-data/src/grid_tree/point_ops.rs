use super::*;

impl<C: GridCell, Co: GridCoord> GridTree<C, Co> {
	pub fn insert(&mut self, pos: &Co::Pos, data: C::Data) -> Option<C::Data> {
		debug_assert!(data <= C::MAX_DATA);
		let pos = Co::to_ivec3(*pos);
		if !self.make_sure_root_covers_pos(pos) {
			return None;
		}
		if !self.has_node_budget() {
			return None;
		}
		self.insert_into_covered(pos, data)
	}

	/// Insert assuming the root already covers `pos` (no root growth).
	pub(super) fn insert_into_covered(&mut self, pos: IVec3, data: C::Data) -> Option<C::Data> {
		let mut current_node_index: u32 = 0;
		let mut current_relative_pos = (pos - self.root_pos).as_uvec3();
		let mut cell_index_stack = [0; MAX_TREE_DEPTH_USIZE];
		let mut cell_index_stack_size = 0;
		let mut current_depth = self.root_depth;
		loop {
			let node = &mut self.nodes[current_node_index as usize];
			let contents_pos = (current_relative_pos / child_size(current_depth)).as_u8vec3();
			let contents_index = get_child_contents_index(contents_pos);
			cell_index_stack[cell_index_stack_size] = contents_index;
			let cell = node.get_child_cell_from_index(contents_index);
			match cell.kind() {
				CellKind::Empty => {
					if current_depth == 0 {
						node.used_cell_count += 1;
						assert!(node.used_cell_count <= 64);
						node.set_child_cell_to_data(contents_pos, data);
						self.try_merge(current_node_index, data, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_none_cell(current_node_index, current_depth, C::data(data), current_relative_pos);
					}
					self.item_count += 1;
					return None;
				}
				CellKind::Data => {
					if cell.data_value() == data {
						return Some(cell.data_value());
					}
					if current_depth == 0 {
						node.set_child_cell_to_data(contents_pos, data);
						self.try_merge(current_node_index, data, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_data_cell(current_node_index, current_depth, cell, C::data(data), current_relative_pos);
					}
					return Some(cell.data_value());
				}
				CellKind::Node => {
					current_relative_pos %= child_size(current_depth);
					current_node_index = cell.node_index();
					current_depth -= 1;
				}
			}
			cell_index_stack_size += 1;
		}
	}

	pub fn remove(&mut self, pos: &Co::Pos) -> Option<C::Data> {
		let pos = Co::to_ivec3(*pos);
		let root_relative_pos = pos - self.root_pos;
		if root_relative_pos.is_negative_bitmask() != 0 {
			return None;
		}
		let root_relative_pos = root_relative_pos.as_uvec3();
		if root_relative_pos.x >= size(self.root_depth)
			|| root_relative_pos.y >= size(self.root_depth)
			|| root_relative_pos.z >= size(self.root_depth)
		{
			return None;
		}
		if !self.has_node_budget() {
			return None;
		}
		let mut current_node_index: u32 = 0;
		let mut current_depth = self.root_depth;
		let mut current_relative_pos = root_relative_pos;
		let mut cell_index_stack = [0; MAX_TREE_DEPTH_USIZE];
		let mut cell_index_stack_size = 0;
		loop {
			let node = &mut self.nodes[current_node_index as usize];
			let contents_pos = (current_relative_pos / child_size(current_depth)).as_u8vec3();
			let contents_index = get_child_contents_index(contents_pos);
			cell_index_stack[cell_index_stack_size] = contents_index;
			let cell = node.get_child_cell_from_index(contents_index);
			match cell.kind() {
				CellKind::Empty => return None,
				CellKind::Data => {
					if current_depth == 0 {
						node.set_child_cell_to_none(contents_pos);
						node.used_cell_count -= 1;
						self.try_merge_empty(current_node_index, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_data_cell(current_node_index, current_depth, cell, C::EMPTY, current_relative_pos);
					}
					self.item_count -= 1;
					return Some(cell.data_value());
				}
				CellKind::Node => {
					current_relative_pos %= child_size(current_depth);
					current_node_index = cell.node_index();
					current_depth -= 1;
				}
			}
			cell_index_stack_size += 1;
		}
	}
}
