use super::*;

impl<C: GridCell, Co: GridCoord> GridTree<C, Co> {
	pub(super) fn build_single_voxel_pairs(&mut self, min: IVec3, max: IVec3, voxels: &[(Co::Pos, C::Data)]) -> bool {
		let Some((root_pos, depth)) = Self::canonical_root_for_bounds(min, max) else {
			return false;
		};
		self.reset_empty_root(root_pos, depth);
		if depth > 2 {
			return false;
		}
		match depth {
			0 => {
				let root = &mut self.nodes[0];
				for (pos, data) in voxels {
					let rel = Co::to_ivec3(*pos) - root_pos;
					let i = (rel.x + rel.y * SIZE as i32 + rel.z * SIZE as i32 * SIZE as i32) as usize;
					if root.contents[i].kind() == CellKind::Empty {
						root.used_cell_count += 1;
						self.item_count += 1;
					}
					root.contents[i] = C::data(*data);
				}
			}
			1 => {
				let mut leaves = vec![GridTreeNode::<C>::new(0); SIZE_USIZE_CUBED];
				for (pos, data) in voxels {
					let rel = Co::to_ivec3(*pos) - root_pos;
					let child = rel / SIZE as i32;
					let leaf_i = (child.x + child.y * SIZE as i32 + child.z * SIZE as i32 * SIZE as i32) as usize;
					let local = rel - child * SIZE as i32;
					let cell_i = (local.x + local.y * SIZE as i32 + local.z * SIZE as i32 * SIZE as i32) as usize;
					let leaf = &mut leaves[leaf_i];
					if leaf.contents[cell_i].kind() == CellKind::Empty {
						leaf.used_cell_count += 1;
						self.item_count += 1;
					}
					leaf.contents[cell_i] = C::data(*data);
				}
				for (i, leaf) in leaves.iter_mut().enumerate() {
					if leaf.used_cell_count == 0 {
						continue;
					}
					let child_index = self.nodes.len() as u32;
					if child_index > C::MAX_NODE_OFFSET {
						return false;
					}
					self.nodes[0].contents[i] = C::node(child_index);
					self.nodes[0].used_cell_count += 1;
					leaf.parent_offset = child_index as u16;
					self.nodes.push(leaf.clone());
				}
			}
			2 => {
				let mut leaves = vec![GridTreeNode::<C>::new(0); SIZE_USIZE_CUBED * SIZE_USIZE_CUBED];
				for (pos, data) in voxels {
					let rel = Co::to_ivec3(*pos) - root_pos;
					let root_child = rel / 16;
					let mid_rel = rel - root_child * 16;
					let mid_child = mid_rel / 4;
					let local = mid_rel - mid_child * 4;
					let root_i = (root_child.x + root_child.y * SIZE as i32 + root_child.z * SIZE as i32 * SIZE as i32) as usize;
					let mid_i = (mid_child.x + mid_child.y * SIZE as i32 + mid_child.z * SIZE as i32 * SIZE as i32) as usize;
					let cell_i = (local.x + local.y * SIZE as i32 + local.z * SIZE as i32 * SIZE as i32) as usize;
					let leaf = &mut leaves[root_i * SIZE_USIZE_CUBED + mid_i];
					if leaf.contents[cell_i].kind() == CellKind::Empty {
						leaf.used_cell_count += 1;
						self.item_count += 1;
					}
					leaf.contents[cell_i] = C::data(*data);
				}
				for root_i in 0..SIZE_USIZE_CUBED {
					if !(0..SIZE_USIZE_CUBED).any(|mid_i| leaves[root_i * SIZE_USIZE_CUBED + mid_i].used_cell_count != 0) {
						continue;
					}
					let mid_index = self.nodes.len() as u32;
					if mid_index > C::MAX_NODE_OFFSET {
						return false;
					}
					self.nodes[0].contents[root_i] = C::node(mid_index);
					self.nodes[0].used_cell_count += 1;
					self.nodes.push(GridTreeNode::new(mid_index as u16));
					for mid_i in 0..SIZE_USIZE_CUBED {
						let leaf_key = root_i * SIZE_USIZE_CUBED + mid_i;
						if leaves[leaf_key].used_cell_count == 0 {
							continue;
						}
						let leaf_index = self.nodes.len() as u32;
						let leaf_offset = leaf_index - mid_index;
						if leaf_index > C::MAX_NODE_OFFSET {
							return false;
						}
						self.nodes[mid_index as usize].contents[mid_i] = C::node(leaf_index);
						self.nodes[mid_index as usize].used_cell_count += 1;
						leaves[leaf_key].parent_offset = leaf_offset as u16;
						self.nodes.push(leaves[leaf_key].clone());
					}
				}
			}
			_ => unreachable!(),
		}
		true
	}

	pub(super) fn build_single_voxel_batch(&mut self, min: IVec3, max: IVec3, ops: &[AreaOp<C::Data>]) -> bool {
		let Some((root_pos, depth)) = Self::canonical_root_for_bounds(min, max) else {
			return false;
		};

		self.reset_empty_root(root_pos, depth);
		if depth <= 2 {
			return self.build_single_voxel_depth2_or_less(depth, root_pos, ops);
		}
		self.build_single_voxel_node(0, depth, min, ops);
		true
	}

	pub(super) fn build_single_voxel_depth2_or_less(&mut self, depth: u8, origin: IVec3, ops: &[AreaOp<C::Data>]) -> bool {
		match depth {
			0 => {
				let root = &mut self.nodes[0];
				for op in ops {
					let rel = op.min - origin;
					let i = (rel.x + rel.y * SIZE as i32 + rel.z * SIZE as i32 * SIZE as i32) as usize;
					if root.contents[i].kind() == CellKind::Empty {
						root.used_cell_count += 1;
						self.item_count += 1;
					}
					root.contents[i] = C::data(op.data);
				}
				true
			}
			1 => {
				let mut leaves = vec![GridTreeNode::<C>::new(0); SIZE_USIZE_CUBED];
				for op in ops {
					let rel = op.min - origin;
					let child = rel / SIZE as i32;
					let leaf_i = (child.x + child.y * SIZE as i32 + child.z * SIZE as i32 * SIZE as i32) as usize;
					let local = rel - child * SIZE as i32;
					let cell_i = (local.x + local.y * SIZE as i32 + local.z * SIZE as i32 * SIZE as i32) as usize;
					let leaf = &mut leaves[leaf_i];
					if leaf.contents[cell_i].kind() == CellKind::Empty {
						leaf.used_cell_count += 1;
						self.item_count += 1;
					}
					leaf.contents[cell_i] = C::data(op.data);
				}
				for (i, leaf) in leaves.iter_mut().enumerate().take(SIZE_USIZE_CUBED) {
					if leaf.used_cell_count == 0 {
						continue;
					}
					let child_index = self.nodes.len() as u32;
					let offset = child_index;
					if child_index > C::MAX_NODE_OFFSET {
						return false;
					}
					self.nodes[0].contents[i] = C::node(child_index);
					self.nodes[0].used_cell_count += 1;
					leaf.parent_offset = offset as u16;
					self.nodes.push(leaf.clone());
				}
				true
			}
			2 => {
				let mut leaves = vec![GridTreeNode::<C>::new(0); SIZE_USIZE_CUBED * SIZE_USIZE_CUBED];
				for op in ops {
					let rel = op.min - origin;
					let root_child = rel / 16;
					let mid_rel = rel - root_child * 16;
					let mid_child = mid_rel / 4;
					let local = mid_rel - mid_child * 4;
					let root_i = (root_child.x + root_child.y * SIZE as i32 + root_child.z * SIZE as i32 * SIZE as i32) as usize;
					let mid_i = (mid_child.x + mid_child.y * SIZE as i32 + mid_child.z * SIZE as i32 * SIZE as i32) as usize;
					let cell_i = (local.x + local.y * SIZE as i32 + local.z * SIZE as i32 * SIZE as i32) as usize;
					let leaf = &mut leaves[root_i * SIZE_USIZE_CUBED + mid_i];
					if leaf.contents[cell_i].kind() == CellKind::Empty {
						leaf.used_cell_count += 1;
						self.item_count += 1;
					}
					leaf.contents[cell_i] = C::data(op.data);
				}

				for root_i in 0..SIZE_USIZE_CUBED {
					let has_root = (0..SIZE_USIZE_CUBED).any(|mid_i| leaves[root_i * SIZE_USIZE_CUBED + mid_i].used_cell_count != 0);
					if !has_root {
						continue;
					}
					let mid_index = self.nodes.len() as u32;
					if mid_index > C::MAX_NODE_OFFSET {
						return false;
					}
					self.nodes[0].contents[root_i] = C::node(mid_index);
					self.nodes[0].used_cell_count += 1;
					self.nodes.push(GridTreeNode::new(mid_index as u16));
					for mid_i in 0..SIZE_USIZE_CUBED {
						let leaf_key = root_i * SIZE_USIZE_CUBED + mid_i;
						if leaves[leaf_key].used_cell_count == 0 {
							continue;
						}
						let leaf_index = self.nodes.len() as u32;
						let leaf_offset = leaf_index - mid_index;
						if leaf_index > C::MAX_NODE_OFFSET {
							return false;
						}
						self.nodes[mid_index as usize].contents[mid_i] = C::node(leaf_index);
						self.nodes[mid_index as usize].used_cell_count += 1;
						leaves[leaf_key].parent_offset = leaf_offset as u16;
						self.nodes.push(leaves[leaf_key].clone());
					}
				}
				true
			}
			_ => unreachable!(),
		}
	}

	pub(super) fn build_single_voxel_node(&mut self, node_index: u32, node_depth: u8, node_origin: IVec3, ops: &[AreaOp<C::Data>]) {
		let cell_size = child_size(node_depth) as i32;
		let mut child_ops: [Vec<AreaOp<C::Data>>; SIZE_USIZE_CUBED] = std::array::from_fn(|_| Vec::new());
		for op in ops {
			let child = (op.min - node_origin).div_euclid(IVec3::splat(cell_size));
			debug_assert!(child.cmpge(IVec3::ZERO).all() && child.cmplt(IVec3::splat(SIZE as i32)).all());
			let i = (child.x + child.y * SIZE as i32 + child.z * SIZE as i32 * SIZE as i32) as usize;
			child_ops[i].push(*op);
		}

		if node_depth == 0 {
			let node = &mut self.nodes[node_index as usize];
			for i in 0..SIZE_CUBED {
				let Some(last) = child_ops[i as usize].last() else { continue };
				node.contents[i as usize] = C::data(last.data);
				node.used_cell_count += 1;
				self.item_count += 1;
			}
			return;
		}

		let mut uniform_children = 0u8;
		let mut uniform_value: Option<C::Data> = None;
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
				let data = bucket[0].data;
				self.nodes[node_index as usize].contents[i as usize] = C::data(data);
				self.nodes[node_index as usize].used_cell_count += 1;
				self.item_count += child_volume as u64;
				uniform_children += 1;
				uniform_value = Some(match uniform_value {
					Some(existing) if existing == data => existing,
					Some(existing) => existing,
					None => data,
				});
				continue;
			}

			let child_index = self.nodes.len() as u32;
			let offset = child_index - node_index;
			debug_assert!(child_index <= C::MAX_NODE_OFFSET);
			self.nodes[node_index as usize].contents[i as usize] = C::node(child_index);
			self.nodes[node_index as usize].used_cell_count += 1;
			self.nodes.push(GridTreeNode::new(offset as u16));
			self.build_single_voxel_node(child_index, node_depth - 1, child_origin, bucket);
		}

		if self.nodes[node_index as usize].used_cell_count == SIZE_CUBED {
			let first = self.nodes[node_index as usize].contents[0];
			if first.kind() == CellKind::Data && self.nodes[node_index as usize].contents.iter().all(|cell| *cell == first) {
				// Keep root as a node so iterator/root invariants match the existing tree,
				// but let parents collapse this child after construction.
				let _ = (uniform_children, uniform_value);
			}
		}
	}

}
