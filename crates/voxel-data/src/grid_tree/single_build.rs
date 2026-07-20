use super::*;

struct TempNode {
	bytes: Vec<u8>,
	occupied: u64,
	used_cell_count: u8,
	stride: usize,
}

impl TempNode {
	fn new(stride: usize) -> Self {
		Self { bytes: vec![0; SIZE_USIZE_CUBED * stride], occupied: 0, used_cell_count: 0, stride }
	}

	fn is_empty(&self) -> bool { self.used_cell_count == 0 }

	fn set_data<G: GridType>(&mut self, grid_type: &G, cell_index: usize, data: G::Data<'_>) {
		let bit = 1u64 << cell_index;
		if self.occupied & bit == 0 {
			self.occupied |= bit;
			self.used_cell_count += 1;
		}
		let start = cell_index * self.stride;
		let slot = &mut self.bytes[start..start + self.stride];
		slot.fill(0);
		grid_type.write_data(data, slot);
	}

	fn write_to_raw<G: GridType>(&self, raw: &mut raw::RawGridTree, grid_type: &G, node_index: u32) {
		raw.set_used_cell_count(node_index, self.used_cell_count);
		for cell_index in 0..SIZE_USIZE_CUBED {
			if self.occupied & (1u64 << cell_index) == 0 {
				continue;
			}
			let start = cell_index * self.stride;
			let data = grid_type.read_data(&self.bytes[start..start + self.stride]);
			raw.set_data(grid_type, node_index, cell_index as u8, data);
		}
	}
}

impl<G: GridType, Co: GridCoord> GridTree<G, Co> {
	pub(super) fn build_single_voxel_pairs<'a>(&mut self, min: IVec3, max: IVec3, voxels: &[(Co::Pos, G::Data<'a>)]) -> bool {
		let Some((root_pos, depth)) = Self::canonical_root_for_bounds(min, max) else {
			return false;
		};
		self.reset_empty_root(root_pos, depth);
		if depth > 2 {
			return false;
		}

		match depth {
			0 => {
				let mut root = TempNode::new(self.raw.cell_stride());
				for (pos, data) in voxels {
					let rel = Co::to_ivec3(*pos) - root_pos;
					let i = (rel.x + rel.y * SIZE as i32 + rel.z * SIZE as i32 * SIZE as i32) as usize;
					root.set_data(&self.grid_type, i, *data);
				}
				self.raw.add_item_count(root.used_cell_count as u64);
				root.write_to_raw(&mut self.raw, &self.grid_type, 0);
			}
			1 => {
				let mut leaves: Vec<_> = (0..SIZE_USIZE_CUBED).map(|_| TempNode::new(self.raw.cell_stride())).collect();
				for (pos, data) in voxels {
					let rel = Co::to_ivec3(*pos) - root_pos;
					let child = rel / SIZE as i32;
					let leaf_i = (child.x + child.y * SIZE as i32 + child.z * SIZE as i32 * SIZE as i32) as usize;
					let local = rel - child * SIZE as i32;
					let cell_i = (local.x + local.y * SIZE as i32 + local.z * SIZE as i32 * SIZE as i32) as usize;
					leaves[leaf_i].set_data(&self.grid_type, cell_i, *data);
				}
				for (i, leaf) in leaves.iter().enumerate() {
					if leaf.is_empty() {
						continue;
					}
					let Some(child_index) = self.alloc_empty_node_after_parent(0) else { return false; };
					self.raw.inc_used_cell_count(0);
					self.raw.set_child_index(0, i as u8, child_index);
					self.raw.add_item_count(leaf.used_cell_count as u64);
					leaf.write_to_raw(&mut self.raw, &self.grid_type, child_index);
				}
			}
			2 => {
				let mut leaves: Vec<_> = (0..SIZE_USIZE_CUBED * SIZE_USIZE_CUBED).map(|_| TempNode::new(self.raw.cell_stride())).collect();
				for (pos, data) in voxels {
					let rel = Co::to_ivec3(*pos) - root_pos;
					let root_child = rel / 16;
					let mid_rel = rel - root_child * 16;
					let mid_child = mid_rel / 4;
					let local = mid_rel - mid_child * 4;
					let root_i = (root_child.x + root_child.y * SIZE as i32 + root_child.z * SIZE as i32 * SIZE as i32) as usize;
					let mid_i = (mid_child.x + mid_child.y * SIZE as i32 + mid_child.z * SIZE as i32 * SIZE as i32) as usize;
					let cell_i = (local.x + local.y * SIZE as i32 + local.z * SIZE as i32 * SIZE as i32) as usize;
					leaves[root_i * SIZE_USIZE_CUBED + mid_i].set_data(&self.grid_type, cell_i, *data);
				}

				for root_i in 0..SIZE_USIZE_CUBED {
					if !(0..SIZE_USIZE_CUBED).any(|mid_i| !leaves[root_i * SIZE_USIZE_CUBED + mid_i].is_empty()) {
						continue;
					}
					let Some(mid_index) = self.alloc_empty_node_after_parent(0) else { return false; };
					self.raw.inc_used_cell_count(0);
					self.raw.set_child_index(0, root_i as u8, mid_index);
					for mid_i in 0..SIZE_USIZE_CUBED {
						let leaf = &leaves[root_i * SIZE_USIZE_CUBED + mid_i];
						if leaf.is_empty() {
							continue;
						}
						let Some(leaf_index) = self.alloc_empty_node_after_parent(mid_index) else { return false; };
						self.raw.inc_used_cell_count(mid_index);
						self.raw.set_child_index(mid_index, mid_i as u8, leaf_index);
						self.raw.add_item_count(leaf.used_cell_count as u64);
						leaf.write_to_raw(&mut self.raw, &self.grid_type, leaf_index);
					}
				}
			}
			_ => unreachable!(),
		}
		true
	}

	pub(super) fn build_single_voxel_batch<'a>(&mut self, min: IVec3, max: IVec3, ops: &[AreaOp<'a, G>]) -> bool {
		let Some((root_pos, depth)) = Self::canonical_root_for_bounds(min, max) else {
			return false;
		};

		self.reset_empty_root(root_pos, depth);
		if depth <= 2 {
			return self.build_single_voxel_depth2_or_less(depth, root_pos, ops);
		}
		self.build_single_voxel_node(0, depth, min, ops)
	}

	pub(super) fn build_single_voxel_depth2_or_less<'a>(&mut self, depth: u8, origin: IVec3, ops: &[AreaOp<'a, G>]) -> bool {
		match depth {
			0 => {
				let mut root = TempNode::new(self.raw.cell_stride());
				for op in ops {
					let rel = op.min - origin;
					let i = (rel.x + rel.y * SIZE as i32 + rel.z * SIZE as i32 * SIZE as i32) as usize;
					root.set_data(&self.grid_type, i, op.data);
				}
				self.raw.add_item_count(root.used_cell_count as u64);
				root.write_to_raw(&mut self.raw, &self.grid_type, 0);
				true
			}
			1 => {
				let mut leaves: Vec<_> = (0..SIZE_USIZE_CUBED).map(|_| TempNode::new(self.raw.cell_stride())).collect();
				for op in ops {
					let rel = op.min - origin;
					let child = rel / SIZE as i32;
					let leaf_i = (child.x + child.y * SIZE as i32 + child.z * SIZE as i32 * SIZE as i32) as usize;
					let local = rel - child * SIZE as i32;
					let cell_i = (local.x + local.y * SIZE as i32 + local.z * SIZE as i32 * SIZE as i32) as usize;
					leaves[leaf_i].set_data(&self.grid_type, cell_i, op.data);
				}
				for (i, leaf) in leaves.iter().enumerate().take(SIZE_USIZE_CUBED) {
					if leaf.is_empty() {
						continue;
					}
					let Some(child_index) = self.alloc_empty_node_after_parent(0) else { return false; };
					self.raw.inc_used_cell_count(0);
					self.raw.set_child_index(0, i as u8, child_index);
					self.raw.add_item_count(leaf.used_cell_count as u64);
					leaf.write_to_raw(&mut self.raw, &self.grid_type, child_index);
				}
				true
			}
			2 => {
				let mut leaves: Vec<_> = (0..SIZE_USIZE_CUBED * SIZE_USIZE_CUBED).map(|_| TempNode::new(self.raw.cell_stride())).collect();
				for op in ops {
					let rel = op.min - origin;
					let root_child = rel / 16;
					let mid_rel = rel - root_child * 16;
					let mid_child = mid_rel / 4;
					let local = mid_rel - mid_child * 4;
					let root_i = (root_child.x + root_child.y * SIZE as i32 + root_child.z * SIZE as i32 * SIZE as i32) as usize;
					let mid_i = (mid_child.x + mid_child.y * SIZE as i32 + mid_child.z * SIZE as i32 * SIZE as i32) as usize;
					let cell_i = (local.x + local.y * SIZE as i32 + local.z * SIZE as i32 * SIZE as i32) as usize;
					leaves[root_i * SIZE_USIZE_CUBED + mid_i].set_data(&self.grid_type, cell_i, op.data);
				}

				for root_i in 0..SIZE_USIZE_CUBED {
					let has_root = (0..SIZE_USIZE_CUBED).any(|mid_i| !leaves[root_i * SIZE_USIZE_CUBED + mid_i].is_empty());
					if !has_root {
						continue;
					}
					let Some(mid_index) = self.alloc_empty_node_after_parent(0) else { return false; };
					self.raw.inc_used_cell_count(0);
					self.raw.set_child_index(0, root_i as u8, mid_index);
					for mid_i in 0..SIZE_USIZE_CUBED {
						let leaf = &leaves[root_i * SIZE_USIZE_CUBED + mid_i];
						if leaf.is_empty() {
							continue;
						}
						let Some(leaf_index) = self.alloc_empty_node_after_parent(mid_index) else { return false; };
						self.raw.inc_used_cell_count(mid_index);
						self.raw.set_child_index(mid_index, mid_i as u8, leaf_index);
						self.raw.add_item_count(leaf.used_cell_count as u64);
						leaf.write_to_raw(&mut self.raw, &self.grid_type, leaf_index);
					}
				}
				true
			}
			_ => unreachable!(),
		}
	}

	pub(super) fn build_single_voxel_node<'a>(&mut self, node_index: u32, node_depth: u8, node_origin: IVec3, ops: &[AreaOp<'a, G>]) -> bool {
		let cell_size = child_size(node_depth) as i32;
		let mut child_ops: [Vec<AreaOp<'a, G>>; SIZE_USIZE_CUBED] = std::array::from_fn(|_| Vec::new());
		for op in ops {
			let child = (op.min - node_origin).div_euclid(IVec3::splat(cell_size));
			debug_assert!(child.cmpge(IVec3::ZERO).all() && child.cmplt(IVec3::splat(SIZE as i32)).all());
			let i = (child.x + child.y * SIZE as i32 + child.z * SIZE as i32 * SIZE as i32) as usize;
			child_ops[i].push(*op);
		}

		if node_depth == 0 {
			for i in 0..SIZE_CUBED {
				let Some(last) = child_ops[i as usize].last() else { continue };
				if self.raw.cell_kind(node_index, i) == CellKind::Empty {
					self.raw.inc_used_cell_count(node_index);
					self.raw.add_item_count(1);
				}
				self.raw.set_data(&self.grid_type, node_index, i, last.data);
			}
			return true;
		}

		for i in 0..SIZE_CUBED {
			let bucket = &child_ops[i as usize];
			if bucket.is_empty() {
				continue;
			}
			let child_origin = node_origin + (get_child_contents_pos(i).as_uvec3() * cell_size as u32).as_ivec3();
			let child_volume = child_size(node_depth) as usize;
			let child_volume = child_volume * child_volume * child_volume;
			let same_data = bucket.iter().all(|op| op.data == bucket[0].data);
			if bucket.len() == child_volume && same_data {
				self.raw.inc_used_cell_count(node_index);
				self.raw.set_data(&self.grid_type, node_index, i, bucket[0].data);
				self.raw.add_item_count(child_volume as u64);
				continue;
			}

			let Some(child_index) = self.alloc_empty_node_after_parent(node_index) else { return false; };
			self.raw.inc_used_cell_count(node_index);
			self.raw.set_child_index(node_index, i, child_index);
			if !self.build_single_voxel_node(child_index, node_depth - 1, child_origin, bucket) {
				return false;
			}
			self.collapse_child_node_if_possible(node_index, i);
		}
		true
	}
}
