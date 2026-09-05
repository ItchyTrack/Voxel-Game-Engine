use super::*;

impl<G: GridType> GridTree<G> {
	/// Insert one data cell. Returns true when the position was already occupied.
	pub fn insert(&mut self, pos: &UVec3, data: G::Data<'_>) -> bool {
		if !self.make_sure_root_covers_pos(*pos) {
			return false;
		}
		if !self.has_node_budget() {
			return false;
		}
		self.insert_into_covered(*pos, data)
	}

	/// Insert assuming the root already covers `pos` (no root growth). Returns true when replacing an occupied cell.
	pub(super) fn insert_into_covered(&mut self, pos: UVec3, data: G::Data<'_>) -> bool {
		let mut current_node_index: u32 = 0;
		let mut current_relative_pos = pos - self.raw.root_pos();
		let mut cell_index_stack = [0; MAX_TREE_DEPTH_USIZE];
		let mut cell_index_stack_size = 0;
		let mut current_depth = self.raw.root_depth();
		loop {
			let contents_pos = (current_relative_pos / child_size(current_depth)).as_u8vec3();
			let contents_index = get_child_contents_index(contents_pos);
			cell_index_stack[cell_index_stack_size] = contents_index;
			match self.raw.cell_kind(current_node_index, contents_index) {
				CellKind::Empty => {
					if current_depth == 0 {
						self.raw.inc_used_cell_count(current_node_index);
						self.raw.set_data(&self.grid_type, current_node_index, contents_index, data);
						self.try_merge(current_node_index, data, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_none_cell(current_node_index, current_depth, data, current_relative_pos);
					}
					self.raw.add_item_count(1);
					return false;
				}
				CellKind::Data => {
					if self.cell_data_eq(current_node_index, contents_index, data) {
						return true;
					}
					if current_depth == 0 {
						self.raw.set_data(&self.grid_type, current_node_index, contents_index, data);
						self.try_merge(current_node_index, data, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_data_cell(current_node_index, current_depth, CellWrite::Data(data), current_relative_pos);
					}
					return true;
				}
				CellKind::Node => {
					current_relative_pos %= child_size(current_depth);
					current_node_index = self.raw.child_index(current_node_index, contents_index);
					current_depth -= 1;
				}
			}
			cell_index_stack_size += 1;
		}
	}

	/// Remove one cell. Returns true when the position was occupied.
	pub fn remove(&mut self, pos: &UVec3) -> bool {
		if pos.cmplt(self.view().root_pos()).any() { return false; }
		let root_relative_pos = pos - self.view().root_pos();
		let root_size = super::size(self.view().root_depth());
		if root_relative_pos.x >= root_size || root_relative_pos.y >= root_size || root_relative_pos.z >= root_size {
			return false;
		}
		if !self.has_node_budget() {
			return false;
		}
		let mut current_node_index: u32 = 0;
		let mut current_depth = self.raw.root_depth();
		let mut current_relative_pos = root_relative_pos;
		let mut cell_index_stack = [0; MAX_TREE_DEPTH_USIZE];
		let mut cell_index_stack_size = 0;
		loop {
			let contents_pos = (current_relative_pos / child_size(current_depth)).as_u8vec3();
			let contents_index = get_child_contents_index(contents_pos);
			cell_index_stack[cell_index_stack_size] = contents_index;
			match self.raw.cell_kind(current_node_index, contents_index) {
				CellKind::Empty => return false,
				CellKind::Data => {
					if current_depth == 0 {
						self.raw.set_empty(current_node_index, contents_index);
						self.raw.dec_used_cell_count(current_node_index);
						self.try_merge_empty(current_node_index, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_data_cell(current_node_index, current_depth, CellWrite::Empty, current_relative_pos);
					}
					self.raw.sub_item_count(1);
					return true;
				}
				CellKind::Node => {
					current_relative_pos %= child_size(current_depth);
					current_node_index = self.raw.child_index(current_node_index, contents_index);
					current_depth -= 1;
				}
			}
			cell_index_stack_size += 1;
		}
	}
}
