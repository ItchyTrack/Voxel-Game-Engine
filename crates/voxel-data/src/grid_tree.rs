use std::fmt::Debug;
use std::hint::unreachable_unchecked;
use std::u16;

use bevy::transform::components::Transform;
use bevy::math::{I8Vec3, I16Vec3, U8Vec3, U16Vec3, Vec3};

pub const LOG_SIZE: u8 = 2;
pub const SIZE: u8 = 1u8 << LOG_SIZE;
pub const SIZE_CUBED: u8 = SIZE * SIZE * SIZE;
pub const SIZE_USIZE: usize = SIZE as usize;
pub const SIZE_USIZE_CUBED: usize = SIZE_USIZE * SIZE_USIZE * SIZE_USIZE;

pub const MAX_TREE_DEPTH: u8 = 10; // it is lower than this but im being safe
pub const MAX_TREE_DEPTH_USIZE: usize = MAX_TREE_DEPTH as usize;

// Highest root depth whose size() still fits in u16 (size(7) = 1<<16 overflows).
const MAX_ROOT_DEPTH: u8 = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridTreeCell {
	// If value == 1<<16-1: NONE, Else if last bit is 0: DATA, Else last bit is 1: NODE.
	// NODE value != 1<<15-1 because then NONE
	pub value: u16,
}

impl GridTreeCell {
	const LAST_BIT_INDEX: u8 = u16::BITS as u8 - 1;
	pub const NONE: GridTreeCell = GridTreeCell{ value: u16::MAX };
	/// You can only use the first 15 bits
	pub fn from_data(data: u16) -> Self {
		Self {
			value: data,
		}
	}
	/// You can only use the first 15 bits and cant be 15 ones
	pub fn from_node_offset(node_index: u16) -> Self {
		Self {
			value: node_index | 1 << Self::LAST_BIT_INDEX,
		}
	}
	pub fn from_raw(value: u16) -> Self {
		Self {
			value: value,
		}
	}
	/// 0: NONE, 1: DATA, 2: NODE
	pub fn value_type(&self) -> u8 {
		if self.value == u16::MAX { return 0; }
		1 + (self.value >> Self::LAST_BIT_INDEX) as u8
	}
	/// Undefined output if NONE
	pub fn value(&self) -> u16 {
		self.value & ((1 << Self::LAST_BIT_INDEX) - 1)
	}
	pub fn value_raw(&self) -> u16 {
		self.value
	}
	pub fn raw_as_data(raw_value: u16) -> u16 {
		raw_value
	}
	pub fn raw_as_node(raw_value: u16) -> u16 {
		raw_value & ((1 << Self::LAST_BIT_INDEX) - 1)
	}
}

#[derive(Debug, Clone)]
pub struct GridTreeNode {
	pub contents: [GridTreeCell; SIZE_USIZE_CUBED],
	pub parent_offset: u16, // if parent_index == 0 then no parent
	pub used_cell_count: u8,
}

pub fn get_child_contents_index(contents_pos: U8Vec3) -> u8 {
	contents_pos.x + contents_pos.y * SIZE + contents_pos.z * SIZE * SIZE
}

pub fn get_child_contents_pos(contents_index: u8) -> U8Vec3 {
	U8Vec3::new(contents_index % SIZE, (contents_index / SIZE) % SIZE, contents_index / (SIZE * SIZE))
}

// fn get_relative_child_pos(child_pos: &I16Vec3, parent_pos: &I16Vec3, child_size: u16) -> U8Vec3 {
// 	((child_pos - parent_pos).as_u16vec3() / child_size).as_u8vec3()
// }

// fn get_parent_pos(child_pos: &I16Vec3, relative_child_pos: U8Vec3, child_size: u16) -> I16Vec3 {
// 	child_pos - (relative_child_pos.as_u16vec3() * child_size).as_i16vec3()
// }

impl GridTreeNode {
	fn new_root() -> Self {
		Self::new(0)
	}
	fn new(parent_offset: u16) -> Self {
		Self {
			contents: [GridTreeCell::NONE; SIZE_USIZE_CUBED],
			parent_offset: parent_offset,
			used_cell_count: 0,
		}
	}
	fn get_parent_offset(&self) -> Option<u16> {
		if self.parent_offset == 0 {
			None
		} else {
			Some(self.parent_offset)
		}
	}
	fn set_parent_offset(&mut self, parent_offset: Option<u16>) {
		match parent_offset {
			Some(parent_offset) => self.parent_offset = parent_offset,
			None => self.parent_offset = 0,
		}
	}
	pub fn size(node_depth: u8) -> u16 {
		1 << (LOG_SIZE * (node_depth + 1))
	}
	pub fn child_size(node_depth: u8) -> u16 {
		1 << (LOG_SIZE * node_depth)
	}
	pub fn parent_size(node_depth: u8) -> u16 {
		1 << (LOG_SIZE * (node_depth + 2))
	}
	// fn child_relative_pos(&self, node_depth: u8, child_contents_pos: U8Vec3) -> U16Vec3 {
	// 	Self::child_size(node_depth) * child_contents_pos.as_u16vec3()
	// }
	// fn parent_relative_pos(&self, node_depth: u8, this_contents_pos: U8Vec3) -> U16Vec3 {
	// 	Self::size(node_depth) * this_contents_pos.as_u16vec3()
	// }
	// (type, value)
	fn get_child_cell_from_index(&self, contents_index: u8) -> (u8, u16) {
		let cell: GridTreeCell = self.contents[contents_index as usize];
		(cell.value_type(), cell.value())
	}
	// (type, value)
	fn get_child_cell(&self, contents_pos: U8Vec3) -> (u8, u16) {
		self.get_child_cell_from_index(get_child_contents_index(contents_pos))
	}
	// fn get_child_cell_raw_from_index(&self, contents_index: u8) -> u16 {
	// 	self.contents[contents_index as usize].value_raw()
	// }
	// fn get_child_cell_raw(&self, contents_pos: U8Vec3) -> u16 {
	// 	self.get_child_cell_raw_from_index(get_child_contents_index(contents_pos))
	// }
	fn get_child_cell_type_and_raw_from_index(&self, contents_index: u8) -> (u8, u16) {
		let cell: GridTreeCell = self.contents[contents_index as usize];
		(cell.value_type(), cell.value_raw())
	}
	fn get_child_cell_type_and_raw(&self, contents_pos: U8Vec3) -> (u8, u16) {
		self.get_child_cell_type_and_raw_from_index(get_child_contents_index(contents_pos))
	}
	fn set_child_cell_from_index(&mut self, contents_index: u8, cell: GridTreeCell) {
		self.contents[contents_index as usize] = cell;
	}
	fn set_child_cell(&mut self, contents_pos: U8Vec3, cell: GridTreeCell) {
		self.set_child_cell_from_index(get_child_contents_index(contents_pos), cell)
	}
	fn set_child_cell_to_none_from_index(&mut self, contents_index: u8) {
		self.set_child_cell_from_index(contents_index, GridTreeCell::NONE);
	}
	fn set_child_cell_to_none(&mut self, contents_pos: U8Vec3) {
		self.set_child_cell_to_none_from_index(get_child_contents_index(contents_pos))
	}
	fn set_child_cell_to_data_from_index(&mut self, contents_index: u8, data: u16) {
		self.set_child_cell_from_index(contents_index, GridTreeCell::from_data(data));
	}
	fn set_child_cell_to_data(&mut self, contents_pos: U8Vec3, data: u16) {
		self.set_child_cell_to_data_from_index(get_child_contents_index(contents_pos), data)
	}
	fn set_child_cell_to_node_from_index(&mut self, contents_index: u8, node_index: u16) {
		self.set_child_cell_from_index(contents_index, GridTreeCell::from_node_offset(node_index));
	}
	fn set_child_cell_to_node(&mut self, contents_pos: U8Vec3, node_index: u16) {
		self.set_child_cell_to_node_from_index(get_child_contents_index(contents_pos), node_index)
	}
}

#[derive(Debug, Clone)]
pub struct GridTree {
	nodes: Vec<GridTreeNode>, // root at 0
	root_pos: I16Vec3,
	root_depth: u8,
	item_count: u64,
	dead_nodes: usize,
}

// A node child reference is the (positive) index distance to the child, stored
// in a cell's low 15 bits (bit 15 is the NODE flag), so it must fit in 1..=0x7FFE
// (0x7FFF would alias the NONE sentinel). The whole arena is bounded by this.
const MAX_NODE_OFFSET: usize = 0x7FFE;

impl GridTree {
	pub fn new() -> Self {
		Self {
			nodes: vec![GridTreeNode::new_root()],
			root_pos: I16Vec3::ZERO,
			root_depth: 0,
			item_count: 0,
			dead_nodes: 0,
		}
	}
	// fn add_child_node(&mut self, parent_index: u32, contents_pos: U8Vec3) -> u16 {
	// 	let next_node_offset = (self.nodes.len() as u32 - parent_index) as u16;
	// 	let parent = &mut self.nodes[parent_index as usize];
	// 	assert!(next_node_offset != 0);
	// 	parent.set_child_cell_to_node(contents_pos, next_node_offset);
	// 	self.nodes.push(GridTreeNode::new(next_node_offset));
	// 	next_node_offset
	// }
	pub fn get(&self, pos: &I16Vec3) -> Option<u16> {
		let root_relative_pos = pos - self.root_pos;
		if root_relative_pos.is_negative_bitmask() != 0 { return None; }
		let root_relative_pos = root_relative_pos.as_u16vec3();
		if root_relative_pos.x >= GridTreeNode::size(self.root_depth) ||
		   root_relative_pos.y >= GridTreeNode::size(self.root_depth) ||
		   root_relative_pos.z >= GridTreeNode::size(self.root_depth) { return None; }
		let mut current_node_index: u32 = 0;
		let mut current_relative_pos = root_relative_pos;
		let mut current_depth = self.root_depth;
		loop {
			let node = &self.nodes[current_node_index as usize];
			let contents_pos = (current_relative_pos / GridTreeNode::child_size(current_depth)).as_u8vec3();
			let cell = node.get_child_cell(contents_pos);
			match cell.0 {
				0 => return None,
				1 => return Some(cell.1),
				2 => {
					current_relative_pos %= GridTreeNode::child_size(current_depth);
					current_node_index += GridTreeCell::raw_as_node(cell.1) as u32;
					current_depth -= 1;
				},
				_ => unsafe { unreachable_unchecked() }
			}
		}
	}
	pub fn contains_key(&self, pos: &I16Vec3) -> bool {
		self.get(pos).is_some()
	}
	pub fn is_area_filled(&self, pos: &I16Vec3, size: &U16Vec3) -> bool {
		for x in 0..size.x {
			for y in 0..size.y {
				for z in 0..size.z {
					if !self.contains_key(&(pos + I16Vec3::new(x as i16, y as i16, z as i16))) {
						return false;
					}
				}
			}
		}
		return true;
	}

	pub fn add_area(&mut self, pos: &I16Vec3, size: &U16Vec3, data: u16) {
		for x in 0..size.x {
			for y in 0..size.y {
				for z in 0..size.z {
					self.insert(&(pos + I16Vec3::new(x as i16, y as i16, z as i16)), data);
				}
			}
		}
	}

	pub fn remove_area(&mut self, pos: &I16Vec3, size: &U16Vec3) {
		for x in 0..size.x {
			for y in 0..size.y {
				for z in 0..size.z {
					self.remove(&(pos + I16Vec3::new(x as i16, y as i16, z as i16)));
				}
			}
		}
	}

	fn make_sure_root_covers_pos(&mut self, pos: &I16Vec3) -> bool {
		if self.nodes[0].used_cell_count == 0 {
			self.nodes = vec![GridTreeNode::new_root()];
			self.root_pos = *pos;
			self.root_depth = 0;
			self.item_count = 0;
			return true;
		}
		if self.root_covers(pos) { return true; }
		self.rebuild_to_cover(*pos)
	}

	fn root_covers(&self, pos: &I16Vec3) -> bool {
		let r = (*pos - self.root_pos).as_ivec3();
		let size = GridTreeNode::size(self.root_depth) as i32;
		r.min_element() >= 0 && r.max_element() < size
	}

	fn rebuild_to_cover(&mut self, pos: I16Vec3) -> bool {
		let mut min = pos;
		let mut max = pos;
		let mut voxels: Vec<(I16Vec3, u16)> = Vec::new();
		for (origin, size, value) in self.iter() {
			min = min.min(origin);
			max = max.max(origin + I16Vec3::splat(size as i16 - 1));
			for dx in 0..size as i16 {
				for dy in 0..size as i16 {
					for dz in 0..size as i16 {
						voxels.push((origin + I16Vec3::new(dx, dy, dz), value));
					}
				}
			}
		}
		let span = (max.as_ivec3() - min.as_ivec3()).max_element() + 1;
		let mut depth = 0u8;
		while (GridTreeNode::size(depth) as i32) < span {
			if depth >= MAX_ROOT_DEPTH {
				bevy::log::warn!("GridTree can't cover span {span} for {pos:?}; skipping insert");
				return false;
			}
			depth += 1;
		}
		self.nodes = vec![GridTreeNode::new_root()];
		self.root_pos = min;
		self.root_depth = depth;
		self.item_count = 0;
		self.dead_nodes = 0;
		for (vp, value) in voxels {
			self.insert_into_covered(&vp, value);
		}
		true
	}
	/// parent depth must be more than 0 and at pos in parent there must be a data cell containing current_cell and current_cell != cell_to_set
	fn set_voxel_in_data_cell(&mut self, parent_node_index: u32, parent_depth: u8, current_cell: GridTreeCell, cell_to_set: GridTreeCell, pos: &U16Vec3) {
		debug_assert!((self.nodes.len() as u32 - parent_node_index) as usize <= MAX_NODE_OFFSET);
		let next_node_offset = (self.nodes.len() as u32 - parent_node_index) as u16;
		let parent = &mut self.nodes[parent_node_index as usize];
		let child_size = GridTreeNode::child_size(parent_depth);
		let relative_pos = (pos / child_size).as_u8vec3();
		assert!(next_node_offset != 0);
		parent.set_child_cell_to_node(relative_pos, next_node_offset);
		self.nodes.push(GridTreeNode {
			contents: [current_cell; SIZE_USIZE_CUBED],
			parent_offset: next_node_offset,
			used_cell_count: SIZE_CUBED,
		});
		if parent_depth == 1 {
			let node = &mut self.nodes[(parent_node_index + next_node_offset as u32) as usize];
			if cell_to_set.value_type() == 0 {
				node.used_cell_count -= 1;
			}
			node.set_child_cell((pos % SIZE as u16).as_u8vec3(), cell_to_set);
		} else {
			self.set_voxel_in_data_cell(parent_node_index + next_node_offset as u32, parent_depth - 1, current_cell, cell_to_set, &(pos % child_size));
		}
	}
	/// parent depth must be more than 0 and at pos in parent there must be a none cell and cell_to_set must be type 1
	fn set_voxel_in_none_cell(&mut self, parent_node_index: u32, parent_depth: u8, cell_to_set: GridTreeCell, pos: &U16Vec3) {
		assert!(cell_to_set.value_type() == 1);
		debug_assert!((self.nodes.len() as u32 - parent_node_index) as usize <= MAX_NODE_OFFSET);
		let next_node_offset = (self.nodes.len() as u32 - parent_node_index) as u16;
		let parent = &mut self.nodes[parent_node_index as usize];
		parent.used_cell_count += 1;
		assert!(parent.used_cell_count <= 64);
		let child_size = GridTreeNode::child_size(parent_depth);
		let contents_pos = (pos / child_size).as_u8vec3();
		assert!(next_node_offset != 0);
		parent.set_child_cell_to_node(contents_pos, next_node_offset);
		self.nodes.push(GridTreeNode {
			contents: [GridTreeCell::NONE; SIZE_USIZE_CUBED],
			parent_offset: next_node_offset,
			used_cell_count: 0,
		});
		if parent_depth == 1 {
			let node = &mut self.nodes[(parent_node_index + next_node_offset as u32) as usize];
			node.used_cell_count += 1;
			node.set_child_cell((pos % SIZE as u16).as_u8vec3(), cell_to_set);
		} else {
			self.set_voxel_in_none_cell(parent_node_index + next_node_offset as u32, parent_depth - 1, cell_to_set, &(pos % child_size));
		}
	}
	pub fn internals(&self) -> (&Vec<GridTreeNode>, I16Vec3, u8) {
		(&self.nodes, self.root_pos, self.root_depth)
	}
	/// cell_to_merge cant be NODE. pos_in_node is any pos
	fn try_merge(&mut self, node_index: u32, data: u16, cell_index_stack: &[u8]) {
		let node = &mut self.nodes[node_index as usize];
		if let Some(parent_offset) = node.get_parent_offset() { // if it dont have a parent it cant be merged
			if node.used_cell_count != SIZE_CUBED {
				return;
			}
			for cell_index in 0..SIZE_CUBED {
				let cell = node.contents[cell_index as usize];
				if cell != GridTreeCell::from_data(data) { return; }
			}
			self.remove_node(node_index);
			let parent_index = node_index - parent_offset as u32;
			self.nodes[parent_index as usize].contents[cell_index_stack[cell_index_stack.len() - 1] as usize] = GridTreeCell::from_data(data);
			self.try_merge(parent_index, data, &cell_index_stack[0..(cell_index_stack.len() - 1)]);
		}
	}
	fn try_merge_empty(&mut self, node_index: u32, cell_index_stack: &[u8]) {
		let node = &mut self.nodes[node_index as usize];
		if let Some(parent_offset) = node.get_parent_offset() { // if it dont have a parent it cant be merged
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
	/// Assumes childern are dead. // Does nothing (leaks the memory)
	pub fn remove_node(&mut self, node_index: u32) {
		if let Some(node) = self.nodes.get_mut(node_index as usize) {
			node.used_cell_count = 255; // mark as deleted
			self.dead_nodes += 1;
		} else {
			println!("NODE GONE!");
		}
	}

	fn maybe_compact(&mut self) {
		if self.dead_nodes * 2 > self.nodes.len() && self.nodes.len() > 64 {
			self.compact();
		}
	}

	/// Rewrite the node arena in DFS order, dropping dead nodes and re-deriving
	/// every child offset. Preserves the logical tree (root_pos/depth, item_count
	/// and all data are unchanged) — only node indices/offsets change.
	fn compact(&mut self) {
		let live = self.nodes.len() - self.dead_nodes;
		let mut new_nodes: Vec<GridTreeNode> = Vec::with_capacity(live);
		new_nodes.push(self.nodes[0].clone()); // root keeps parent_offset 0
		let mut stack: Vec<(u32, u32)> = vec![(0, 0)]; // (old_index, new_index)
		while let Some((old_idx, new_idx)) = stack.pop() {
			for ci in 0..SIZE_CUBED {
				let cell = self.nodes[old_idx as usize].contents[ci as usize];
				if cell.value_type() == 2 {
					let child_old = old_idx + GridTreeCell::raw_as_node(cell.value_raw()) as u32;
					let child_new = new_nodes.len() as u32;
					let mut child = self.nodes[child_old as usize].clone();
					child.parent_offset = (child_new - new_idx) as u16;
					new_nodes.push(child);
					new_nodes[new_idx as usize].set_child_cell_to_node_from_index(ci, (child_new - new_idx) as u16);
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
	fn has_node_budget(&mut self) -> bool {
		self.maybe_compact();
		if self.nodes.len() + MAX_TREE_DEPTH_USIZE <= MAX_NODE_OFFSET {
			return true;
		}
		bevy::log::warn!("GridTree node arena full ({} live); skipping edit", self.nodes.len());
		false
	}

	pub fn insert(&mut self, pos: &I16Vec3, data: u16) -> Option<u16> {
		if !self.make_sure_root_covers_pos(pos) { return None; }
		if !self.has_node_budget() { return None; }
		self.insert_into_covered(pos, data)
	}

	/// Insert assuming the root already covers `pos` (no root growth).
	fn insert_into_covered(&mut self, pos: &I16Vec3, data: u16) -> Option<u16> {
		let mut current_node_index: u32 = 0;
		let mut current_relative_pos = (pos - self.root_pos).as_u16vec3();
		let mut cell_index_stack = [0; MAX_TREE_DEPTH_USIZE];
		let mut cell_index_stack_size = 0;
		let mut current_depth = self.root_depth;
		loop {
			let node = &mut self.nodes[current_node_index as usize];
			let contents_pos = (current_relative_pos / GridTreeNode::child_size(current_depth)).as_u8vec3();
			let contents_index = get_child_contents_index(contents_pos);
			cell_index_stack[cell_index_stack_size] = contents_index;
			let cell = node.get_child_cell_type_and_raw_from_index(contents_index);
			match cell.0 {
				0 => {
					if current_depth == 0 {
						node.used_cell_count += 1;
						assert!(node.used_cell_count <= 64);
						if node.used_cell_count > 64 {
							panic!("ERROR!!!");
						}
						node.set_child_cell_to_data(contents_pos, data);
						self.try_merge(current_node_index, data, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_none_cell(current_node_index, current_depth, GridTreeCell::from_data(data), &current_relative_pos);
					}
					self.item_count += 1;
					return None;
				},
				1 => {
					if GridTreeCell::raw_as_data(cell.1) == data {
						return Some(GridTreeCell::raw_as_data(cell.1));
					}
					if current_depth == 0 {
						node.set_child_cell_to_data(contents_pos, data);
						self.try_merge(current_node_index, data, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_data_cell(current_node_index, current_depth, GridTreeCell::from_raw(cell.1), GridTreeCell::from_data(data), &current_relative_pos);
					}
					return Some(GridTreeCell::raw_as_data(cell.1));
				},
				2 => {
					current_relative_pos %= GridTreeNode::child_size(current_depth);
					current_node_index += GridTreeCell::raw_as_node(cell.1) as u32;
					current_depth -= 1;
				},
				_ => unsafe { unreachable_unchecked() }
			}
			cell_index_stack_size += 1;
		}
	}

	pub fn remove(&mut self, pos: &I16Vec3) -> Option<u16> {
		let root_relative_pos = pos - self.root_pos;
		if root_relative_pos.is_negative_bitmask() != 0 { return None; }
		let root_relative_pos = root_relative_pos.as_u16vec3();
		if root_relative_pos.x >= GridTreeNode::size(self.root_depth) ||
		   root_relative_pos.y >= GridTreeNode::size(self.root_depth) ||
		   root_relative_pos.z >= GridTreeNode::size(self.root_depth) { return None; }
		if !self.has_node_budget() { return None; }
		let mut current_node_index: u32 = 0;
		let mut current_depth = self.root_depth;
		let mut current_relative_pos = root_relative_pos;
		let mut cell_index_stack = [0; MAX_TREE_DEPTH_USIZE];
		let mut cell_index_stack_size = 0;
		loop {
			let node = &mut self.nodes[current_node_index as usize];
			let contents_pos = (current_relative_pos / GridTreeNode::child_size(current_depth)).as_u8vec3();
			let contents_index = get_child_contents_index(contents_pos);
			cell_index_stack[cell_index_stack_size] = contents_index;
			let cell = node.get_child_cell_type_and_raw_from_index(contents_index);
			match cell.0 {
				0 => return None,
				1 => {
					if current_depth == 0 {
						node.set_child_cell_to_none(contents_pos);
						node.used_cell_count -= 1;
						self.try_merge_empty(current_node_index, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_data_cell(current_node_index, current_depth, GridTreeCell::from_raw(cell.1), GridTreeCell::NONE, &current_relative_pos);
					}
					self.item_count -= 1;
					return Some(cell.1);
				},
				2 => {
					current_relative_pos %= GridTreeNode::child_size(current_depth);
					current_node_index += GridTreeCell::raw_as_node(cell.1) as u32;
					current_depth -= 1;
				},
				_ => unsafe { unreachable_unchecked() }
			}
			cell_index_stack_size += 1;
		}
	}

	pub fn len(&self) -> u64 { return self.item_count; }
	pub fn is_empty(&self) -> bool { return self.item_count == 0; }
	pub fn iter(&self) -> GridTreeIterator<'_> { GridTreeIterator::new(self) }

	fn ray_aabb_intersection(start: &Vec3, direction: &Vec3, aabb: &(Vec3, Vec3)) -> Option<f32> {
		let (min, max) = aabb;

		if start.cmpge(*min).all() && start.cmple(*max).all() {
			return Some(0.0);
		}

		let inv = Vec3::ONE / *direction;
		let t1 = (*min - *start) * inv;
		let t2 = (*max - *start) * inv;

		let tmin = t1.min(t2).max_element();
		let tmax = t1.max(t2).min_element();

		if tmax < 0.0 || tmin > tmax { return None; }

		Some(tmin)
	}

	pub fn raycast(&self, transform: &Transform, max_length: Option<f32>/*, debug_transform: &Transform*/) -> Option<(I16Vec3, I8Vec3, f32)> {
		let max_length = max_length.unwrap_or(f32::MAX);

		let origin = transform.translation;
		let dir = transform.rotation * Vec3::Z;

		// debug_draw::line(debug_transform * (origin), debug_transform * (origin + dir * 30.0), &Vec4::new(0.0, 0.0, 0.0, 1.0));

		let root_min = self.root_pos.as_vec3();
		let root_max = root_min + Vec3::splat(GridTreeNode::size(self.root_depth) as f32);

		// Ray vs root AABB
		let distance_to_aabb = match Self::ray_aabb_intersection(&origin, &dir, &(root_min, root_max)) {
			Some(dis) => dis,
			None => return None,
		};
		let post_aabb_origin_pre_shift = origin + dir * distance_to_aabb;
		let post_aabb_origin = post_aabb_origin_pre_shift
			.min(self.root_pos.as_vec3() + Vec3::splat(((GridTreeNode::size(self.root_depth)) as f32) - 0.00001))
			.max(self.root_pos.as_vec3());
		let post_aabb_origin = post_aabb_origin.move_towards(post_aabb_origin.floor() + 0.5, 0.001);
		let root_relative_post_aabb_origin = post_aabb_origin - self.root_pos.as_vec3();
		let delta = dir.recip().abs();
		let step = dir.signum().as_i8vec3();
		let mut axis_distances = dir.recip() * Vec3::new(
			if step.x > 0 { root_relative_post_aabb_origin.x.ceil() } else { root_relative_post_aabb_origin.x.floor() } - root_relative_post_aabb_origin.x,
			if step.y > 0 { root_relative_post_aabb_origin.y.ceil() } else { root_relative_post_aabb_origin.y.floor() } - root_relative_post_aabb_origin.y,
			if step.z > 0 { root_relative_post_aabb_origin.z.ceil() } else { root_relative_post_aabb_origin.z.floor() } - root_relative_post_aabb_origin.z,
		) + distance_to_aabb;
		// debug_draw::line(debug_transform * (origin + Vec3::new(0.0, 0.025, 0.0)), debug_transform * (origin + Vec3::new(0.0, 0.025, 0.0) + dir * axis_distances.x), &Vec4::new(1.0, 0.2, 0.2, 1.0));
		// debug_draw::line(debug_transform * (origin + Vec3::new(0.0, 0.05, 0.0)), debug_transform * (origin + Vec3::new(0.0, 0.05, 0.0) + dir * axis_distances.y), &Vec4::new(0.2, 1.0, 0.2, 1.0));
		// debug_draw::line(debug_transform * (origin + Vec3::new(0.0, 0.075, 0.0)), debug_transform * (origin + Vec3::new(0.0, 0.075, 0.0) + dir * axis_distances.z), &Vec4::new(0.2, 0.2, 1.0, 1.0));
		// debug_draw::point(debug_transform * (root_relative_post_aabb_origin + self.root_pos.as_vec3()), &Vec4::W, 1.0);
		let mut root_relative_grid_pos = root_relative_post_aabb_origin.as_u16vec3();
		let mut last_step_axis = (post_aabb_origin_pre_shift - post_aabb_origin).abs().max_position() as u8;
		let mut current_node_index = 0u32;
		let mut current_depth = self.root_depth;
		let mut last_distance = distance_to_aabb;
		if max_length < last_distance { return None; }
		loop {
			let mut current_node = &self.nodes[current_node_index as usize];
			// debug_draw::rectangular_prism(&(debug_transform * Transform::from_translation((self.root_pos + root_relative_grid_pos.as_i16vec3()).as_vec3())), Vec3::ONE, &Vec4::ONE, false);
			let node_relative_grid_pos = root_relative_grid_pos % GridTreeNode::size(current_depth);
			// debug_draw::rectangular_prism(&(debug_transform * Transform::from_translation((
			// 	self.root_pos + root_relative_grid_pos.as_i16vec3() - node_relative_grid_pos.as_i16vec3()
			// ).as_vec3())), Vec3::splat(current_node.size() as f32), &Vec4::new(0.0, 0.0, 1.0, 1.0), false);
			let contents_pos = (node_relative_grid_pos / GridTreeNode::child_size(current_depth)).as_u8vec3();
			let cell = current_node.get_child_cell(contents_pos);
			match cell.0 {
				0 => { // NONE
					if GridTreeNode::child_size(current_depth) != 1 {
						let node_cell_relative_grid_pos = node_relative_grid_pos % GridTreeNode::child_size(current_depth);
						let mut step_amount = U16Vec3::select(
							step.cmpgt(I8Vec3::ZERO),
							U16Vec3::splat(GridTreeNode::child_size(current_depth) - 1) - node_cell_relative_grid_pos,
							node_cell_relative_grid_pos
						);
						// step to edge of cell. step_amount is 0 if child_size is 0
						let distance_to_edge_of_cell = axis_distances + step_amount.as_vec3() * delta;
						match distance_to_edge_of_cell.min_position() {
							0 => {
								step_amount.y = ((distance_to_edge_of_cell.x - axis_distances.y + delta.y) / delta.y).abs() as u16;
								step_amount.z = ((distance_to_edge_of_cell.x - axis_distances.z + delta.z) / delta.z).abs() as u16;
							},
							1 => {
								step_amount.x = ((distance_to_edge_of_cell.y - axis_distances.x + delta.x) / delta.x).abs() as u16;
								step_amount.z = ((distance_to_edge_of_cell.y - axis_distances.z + delta.z) / delta.z).abs() as u16;
							},
							2 => {
								step_amount.x = ((distance_to_edge_of_cell.z - axis_distances.x + delta.x) / delta.x).abs() as u16;
								step_amount.y = ((distance_to_edge_of_cell.z - axis_distances.y + delta.y) / delta.y).abs() as u16;
							},
							_ => unsafe { unreachable_unchecked(); }
						}
						axis_distances += delta * step_amount.as_vec3();
						root_relative_grid_pos = (root_relative_grid_pos.as_i16vec3() + step_amount.as_i16vec3() * step.as_i16vec3()).as_u16vec3();
						// debug_draw::rectangular_prism(&(debug_transform * Transform::from_translation((self.root_pos + root_relative_grid_pos.as_i16vec3()).as_vec3())), Vec3::ONE, &Vec4::W, false);
						// println!("{root_relative_grid_pos}");
					}
					match axis_distances.min_position() {
						0 => {
							if max_length < axis_distances.x { return None; }
							let root_relative_grid_pos_x = root_relative_grid_pos.x as i16 + step.x as i16;
							if root_relative_grid_pos_x < 0 || root_relative_grid_pos_x >= GridTreeNode::size(self.root_depth) as i16 { return None; }
							let root_relative_grid_pos_x = root_relative_grid_pos_x as u16;
							loop {
								if root_relative_grid_pos.x / GridTreeNode::size(current_depth) == root_relative_grid_pos_x / GridTreeNode::size(current_depth) {
									break;
								}
								current_depth += 1;
								let parent_offset = current_node.get_parent_offset()?;
								current_node_index -= parent_offset as u32;
								current_node = &self.nodes[current_node_index as usize];
							}
							root_relative_grid_pos.x = root_relative_grid_pos_x;
							last_distance = axis_distances.x;
							axis_distances.x += delta.x;
							last_step_axis = 0;
						},
						1 => {
							if max_length < axis_distances.y { return None; }
							let root_relative_grid_pos_y = root_relative_grid_pos.y as i16 + step.y as i16;
							if root_relative_grid_pos_y < 0 || root_relative_grid_pos_y >= GridTreeNode::size(self.root_depth) as i16 { return None; }
							let root_relative_grid_pos_y = root_relative_grid_pos_y as u16;
							loop {
								if root_relative_grid_pos.y / GridTreeNode::size(current_depth) == root_relative_grid_pos_y / GridTreeNode::size(current_depth) {
									break;
								}
								current_depth += 1;
								let parent_offset = current_node.get_parent_offset()?;
								current_node_index -= parent_offset as u32;
								current_node = &self.nodes[current_node_index as usize];
							}
							root_relative_grid_pos.y = root_relative_grid_pos_y;
							last_distance = axis_distances.y;
							axis_distances.y += delta.y;
							last_step_axis = 1;
						},
						2 => {
							if max_length < axis_distances.z { return None; }
							let root_relative_grid_pos_z = root_relative_grid_pos.z as i16 + step.z as i16;
							if root_relative_grid_pos_z < 0 || root_relative_grid_pos_z >= GridTreeNode::size(self.root_depth) as i16 { return None; }
							let root_relative_grid_pos_z = root_relative_grid_pos_z as u16;
							loop {
								if root_relative_grid_pos.z / GridTreeNode::size(current_depth) == root_relative_grid_pos_z / GridTreeNode::size(current_depth) {
									break;
								}
								current_depth += 1;
								let parent_offset = current_node.get_parent_offset()?;
								current_node_index -= parent_offset as u32;
								current_node = &self.nodes[current_node_index as usize];
							}
							root_relative_grid_pos.z = root_relative_grid_pos_z;
							last_distance = axis_distances.z;
							axis_distances.z += delta.z;
							last_step_axis = 2;
						},
						_ => unsafe { unreachable_unchecked(); }
					}
				},
				1 => { // DATA
					return Some((
						root_relative_grid_pos.as_i16vec3() + self.root_pos,
						-step.to_array()[last_step_axis as usize] * I8Vec3::AXES[last_step_axis as usize],
						last_distance
					));
				},
				2 => { // NODE
					current_depth -= 1;
					current_node_index += cell.1 as u32;
				}
				_ => unsafe { unreachable_unchecked(); }
			}
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use std::collections::HashMap;
	use bevy::math::Vec3;
	use bevy::transform::components::Transform;

	fn lcg(state: &mut u64) -> u64 {
		*state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
		*state >> 33
	}

	fn p(x: i16, y: i16, z: i16) -> I16Vec3 { I16Vec3::new(x, y, z) }

	/// Expand the tree's iterator into a flat voxel map, asserting cells never overlap.
	fn tree_voxels(tree: &GridTree) -> HashMap<I16Vec3, u16> {
		let mut m = HashMap::new();
		for (origin, size, value) in tree.iter() {
			for dx in 0..size as i16 {
				for dy in 0..size as i16 {
					for dz in 0..size as i16 {
						let pos = origin + I16Vec3::new(dx, dy, dz);
						assert!(m.insert(pos, value).is_none(), "iter overlapped at {pos:?}");
					}
				}
			}
		}
		m
	}

	/// Ground-truth check: get(), iter() expansion, and len() all agree with the oracle.
	fn assert_matches_oracle(tree: &GridTree, oracle: &HashMap<I16Vec3, u16>) {
		for (pos, v) in oracle {
			assert_eq!(tree.get(pos), Some(*v), "get({pos:?}) mismatch");
		}
		assert_eq!(&tree_voxels(tree), oracle, "iter expansion differs from oracle");
		assert_eq!(tree.len(), oracle.len() as u64, "len() mismatch");
	}

	// ---- basic API ----

	#[test]
	fn new_tree_is_empty() {
		let t = GridTree::new();
		assert!(t.is_empty());
		assert_eq!(t.len(), 0);
		assert_eq!(t.get(&p(0, 0, 0)), None);
		assert_eq!(t.get(&p(-5, 9, 1000)), None);
		assert!(!t.contains_key(&p(0, 0, 0)));
	}

	#[test]
	fn insert_then_get() {
		let mut t = GridTree::new();
		assert_eq!(t.insert(&p(3, 4, 5), 7), None);
		assert_eq!(t.get(&p(3, 4, 5)), Some(7));
		assert!(t.contains_key(&p(3, 4, 5)));
		assert_eq!(t.len(), 1);
		assert!(!t.is_empty());
	}

	#[test]
	fn insert_overwrite_returns_old_value() {
		let mut t = GridTree::new();
		t.insert(&p(1, 1, 1), 10);
		assert_eq!(t.insert(&p(1, 1, 1), 20), Some(10));
		assert_eq!(t.get(&p(1, 1, 1)), Some(20));
		assert_eq!(t.len(), 1, "overwrite must not change len");
	}

	#[test]
	fn insert_same_value_returns_old_and_noops() {
		let mut t = GridTree::new();
		t.insert(&p(2, 2, 2), 9);
		assert_eq!(t.insert(&p(2, 2, 2), 9), Some(9));
		assert_eq!(t.len(), 1);
	}

	#[test]
	fn remove_returns_value_absent_is_none() {
		let mut t = GridTree::new();
		t.insert(&p(5, 6, 7), 42);
		assert_eq!(t.remove(&p(0, 0, 0)), None);
		assert_eq!(t.remove(&p(5, 6, 7)), Some(42));
		assert_eq!(t.get(&p(5, 6, 7)), None);
		assert!(t.is_empty());
		assert_eq!(t.remove(&p(5, 6, 7)), None);
	}

	#[test]
	fn negative_coordinates_roundtrip() {
		let mut t = GridTree::new();
		t.insert(&p(-10, -20, -30), 3);
		t.insert(&p(-1, -1, -1), 4);
		t.insert(&p(-10, 5, -30), 5);
		assert_eq!(t.get(&p(-10, -20, -30)), Some(3));
		assert_eq!(t.get(&p(-1, -1, -1)), Some(4));
		assert_eq!(t.get(&p(-10, 5, -30)), Some(5));
		assert_eq!(t.len(), 3);
	}

	// ---- merging behavior ----

	#[test]
	fn root_node_never_merges() {
		// A uniform SIZE^3 cube that IS the whole tree stays unmerged: try_merge
		// requires a parent, and the root has none. So it yields SIZE^3 leaves.
		let mut t = GridTree::new();
		for x in 0..SIZE as i16 {
			for y in 0..SIZE as i16 {
				for z in 0..SIZE as i16 {
					t.insert(&p(x, y, z), 1);
				}
			}
		}
		assert_eq!(t.iter().count() as u64, (SIZE as u64).pow(3));
		assert!(t.iter().all(|(_, size, _)| size == 1));
		assert_eq!(t.len(), (SIZE as u64).pow(3));
	}

	#[test]
	fn full_child_node_merges_into_single_cell() {
		// With depth forced to 1 (anchor in another child cell), a uniformly
		// filled child cell collapses to one data cell of side SIZE.
		let mut t = GridTree::new();
		t.insert(&p(0, 0, 0), 1); // sets root_pos = origin (avoids negative-side growth)
		t.insert(&p(2 * SIZE as i16, 0, 0), 2); // anchor: forces root to depth 1
		for x in 0..SIZE as i16 {
			for y in 0..SIZE as i16 {
				for z in 0..SIZE as i16 {
					t.insert(&p(x, y, z), 1);
				}
			}
		}
		let cells: Vec<_> = t.iter().collect();
		assert!(
			cells.contains(&(p(0, 0, 0), SIZE as u16, 1)),
			"uniform child cell should merge to one size-{SIZE} cell, got {cells:?}"
		);
		assert_eq!(t.len(), (SIZE as u64).pow(3) + 1);
	}

	#[test]
	fn mixed_values_do_not_merge() {
		let mut t = GridTree::new();
		for x in 0..SIZE as i16 {
			for y in 0..SIZE as i16 {
				for z in 0..SIZE as i16 {
					t.insert(&p(x, y, z), if (x + y + z) % 2 == 0 { 1 } else { 2 });
				}
			}
		}
		let cells: Vec<_> = t.iter().collect();
		assert!(cells.len() > 1, "non-uniform cube must not merge");
		assert!(cells.iter().all(|(_, size, _)| *size == 1));
	}

	#[test]
	fn remove_from_merged_region_splits_it() {
		let mut t = GridTree::new();
		t.insert(&p(0, 0, 0), 1);
		t.insert(&p(2 * SIZE as i16, 0, 0), 2); // force depth 1 so the cube merges
		for x in 0..SIZE as i16 {
			for y in 0..SIZE as i16 {
				for z in 0..SIZE as i16 {
					t.insert(&p(x, y, z), 1);
				}
			}
		}
		assert!(t.iter().any(|c| c == (p(0, 0, 0), SIZE as u16, 1)), "precondition: merged");
		assert_eq!(t.remove(&p(1, 2, 3)), Some(1));
		assert_eq!(t.get(&p(1, 2, 3)), None);
		assert_eq!(t.get(&p(0, 0, 0)), Some(1)); // neighbours survive the split
		assert_eq!(t.len(), (SIZE as u64).pow(3) - 1 + 1);
	}

	// ---- area helpers ----

	#[test]
	fn add_area_then_is_area_filled() {
		let mut t = GridTree::new();
		t.add_area(&p(0, 0, 0), &U16Vec3::new(5, 5, 5), 8);
		assert!(t.is_area_filled(&p(0, 0, 0), &U16Vec3::new(5, 5, 5)));
		assert!(!t.is_area_filled(&p(0, 0, 0), &U16Vec3::new(6, 5, 5)));
		assert_eq!(t.len(), 125);
	}

	#[test]
	fn remove_area_clears_region() {
		let mut t = GridTree::new();
		t.add_area(&p(0, 0, 0), &U16Vec3::new(8, 8, 8), 1);
		t.remove_area(&p(2, 2, 2), &U16Vec3::new(3, 3, 3));
		assert_eq!(t.get(&p(3, 3, 3)), None);
		assert_eq!(t.get(&p(0, 0, 0)), Some(1));
		assert_eq!(t.len(), 8 * 8 * 8 - 3 * 3 * 3);
	}

	// ---- iteration ----

	#[test]
	fn iter_covers_all_inserted_voxels() {
		let mut t = GridTree::new();
		let mut oracle = HashMap::new();
		for &(x, y, z, v) in &[(0i16, 0i16, 0i16, 1u16), (10, 0, 0, 2), (0, 31, 0, 3), (5, 5, 5, 4), (-7, 2, 9, 5)] {
			t.insert(&p(x, y, z), v);
			oracle.insert(p(x, y, z), v);
		}
		assert_matches_oracle(&t, &oracle);
	}

	// ---- raycast ----

	#[test]
	fn raycast_hits_voxel_along_z() {
		let mut t = GridTree::new();
		t.insert(&p(0, 0, 10), 1);
		let tf = Transform {
			translation: Vec3::new(0.5, 0.5, -1.0),
			rotation: bevy::math::Quat::from_rotation_arc(Vec3::Z, Vec3::Z),
			scale: Vec3::ONE,
		};
		let hit = t.raycast(&tf, None);
		assert!(hit.is_some(), "ray straight at voxel should hit");
		let (pos, _normal, dist) = hit.unwrap();
		assert_eq!(pos, p(0, 0, 10));
		assert!(dist > 0.0);
	}

	#[test]
	fn raycast_miss_returns_none() {
		let mut t = GridTree::new();
		t.insert(&p(0, 0, 10), 1);
		let tf = Transform {
			translation: Vec3::new(0.5, 0.5, -1.0),
			rotation: bevy::math::Quat::from_rotation_arc(Vec3::Z, -Vec3::Z),
			scale: Vec3::ONE,
		};
		assert_eq!(t.raycast(&tf, None), None);
	}

	#[test]
	fn raycast_empty_tree_is_none() {
		let t = GridTree::new();
		let tf = Transform::from_translation(Vec3::ZERO);
		assert_eq!(t.raycast(&tf, None), None);
	}

	#[test]
	fn merge_cascades_through_levels() {
		// A uniform 16^3 region (depth-2 tree) collapses each depth-1 subtree, so
		// every yielded cell has side > 1 and far fewer cells than voxels.
		let n = (SIZE as i16) * (SIZE as i16); // 16
		let mut t = GridTree::new();
		let mut oracle = HashMap::new();
		for x in 0..n { for y in 0..n { for z in 0..n {
			t.insert(&p(x, y, z), 7);
			oracle.insert(p(x, y, z), 7);
		}}}
		assert_matches_oracle(&t, &oracle);
		let cells: Vec<_> = t.iter().collect();
		assert!(cells.iter().all(|(_, size, _)| *size > 1), "interior should merge");
		assert!((cells.len() as u64) < oracle.len() as u64, "merging must reduce cell count");
	}

	#[test]
	fn overwrite_single_voxel_in_merged_region() {
		let mut t = GridTree::new();
		t.insert(&p(0, 0, 0), 1);
		t.insert(&p(2 * SIZE as i16, 0, 0), 9); // depth 1
		for x in 0..SIZE as i16 { for y in 0..SIZE as i16 { for z in 0..SIZE as i16 {
			t.insert(&p(x, y, z), 1);
		}}}
		// Splitting a merged cell by overwriting one voxel keeps the rest intact.
		assert_eq!(t.insert(&p(2, 2, 2), 5), Some(1));
		assert_eq!(t.get(&p(2, 2, 2)), Some(5));
		assert_eq!(t.get(&p(1, 1, 1)), Some(1));
		assert_eq!(t.len(), (SIZE as u64).pow(3) + 1);
	}

	#[test]
	fn get_outside_root_bounds_is_none() {
		let mut t = GridTree::new();
		t.insert(&p(0, 0, 0), 1);
		assert_eq!(t.get(&p(10_000, 0, 0)), None);
		assert_eq!(t.get(&p(-10_000, 0, 0)), None);
		assert_eq!(t.get(&p(0, 5, 0)), None);
	}

	#[test]
	fn extreme_coordinates_do_not_panic() {
		// i16 extremes must not panic or hang; the depth cap may skip them, which
		// is acceptable as long as it stays self-consistent.
		let mut t = GridTree::new();
		t.insert(&p(0, 0, 0), 1);
		t.insert(&p(i16::MAX, i16::MAX, i16::MAX), 2);
		t.insert(&p(i16::MIN, i16::MIN, i16::MIN), 3);
		// Whatever was actually stored must read back consistently.
		let voxels = tree_voxels(&t);
		for (pos, v) in &voxels {
			assert_eq!(t.get(pos), Some(*v));
		}
	}

	#[test]
	fn raycast_reports_entry_face_normal() {
		let mut t = GridTree::new();
		t.insert(&p(0, 0, 10), 1);
		let tf = Transform {
			translation: Vec3::new(0.5, 0.5, -1.0),
			rotation: bevy::math::Quat::from_rotation_arc(Vec3::Z, Vec3::Z),
			scale: Vec3::ONE,
		};
		let (pos, normal, _) = t.raycast(&tf, None).unwrap();
		assert_eq!(pos, p(0, 0, 10));
		assert_eq!(normal, bevy::math::I8Vec3::new(0, 0, -1), "hit -Z face entering along +Z");
	}

	#[test]
	fn raycast_respects_max_length() {
		let mut t = GridTree::new();
		t.insert(&p(0, 0, 100), 1);
		let tf = Transform {
			translation: Vec3::new(0.5, 0.5, 0.0),
			rotation: bevy::math::Quat::from_rotation_arc(Vec3::Z, Vec3::Z),
			scale: Vec3::ONE,
		};
		assert!(t.raycast(&tf, Some(10.0)).is_none(), "voxel beyond max_length must miss");
		assert!(t.raycast(&tf, Some(200.0)).is_some());
	}

	// Characterizes the KNOWN node leak: remove_node never reclaims slots, so the
	// node Vec grows under churn that creates/collapses nodes (without emptying
	// the tree, which would reset the root). Fixing the leak should flip this.
	#[test]
	fn node_arena_bounded_under_churn() {
		// Compaction must reclaim dead nodes so the arena does not grow without
		// bound under repeated create/collapse churn (the old remove_node leak).
		let mut t = GridTree::new();
		// Spaced base voxels: each sits alone in its leaf, so toggling it actually
		// collapses (and would leak) a node.
		for i in 0..16i16 { for j in 0..16i16 {
			t.insert(&p(i * 4, j * 4, ((i + j) % 16) * 4), 1);
		}}
		let baseline = t.internals().0.len();
		for _ in 0..400 {
			for i in 0..16i16 { for j in 0..16i16 {
				let q = p(i * 4, j * 4, ((i + j) % 16) * 4);
				t.remove(&q);
				t.insert(&q, 2);
			}}
		}
		let after = t.internals().0.len();
		assert!(after <= baseline * 2, "arena must stay bounded by compaction ({baseline} -> {after})");
	}

	#[test]
	fn node_cap_is_handled_gracefully() {
		// Past the 15-bit node-offset limit, edits are skipped (warned), never
		// corrupting: everything the tree reports as stored must read back.
		let mut t = GridTree::new();
		let mut s = 0xC0FFEEu64;
		for _ in 0..400_000 {
			let q = p(
				(lcg(&mut s) % 8000) as i16,
				(lcg(&mut s) % 8000) as i16,
				(lcg(&mut s) % 8000) as i16,
			);
			t.insert(&q, 1);
		}
		// Whatever actually made it in must be self-consistent.
		for (pos, v) in &tree_voxels(&t) {
			assert_eq!(t.get(pos), Some(*v));
		}
	}

	// ---- differential model tests vs HashMap oracle ----

	fn run_model(lo: i16, hi: i16, n_ops: usize, seed: u64) {
		let mut t = GridTree::new();
		let mut oracle: HashMap<I16Vec3, u16> = HashMap::new();
		let mut s = seed;
		let span = (hi - lo) as u64;
		for op in 0..n_ops {
			let pos = p(
				lo + (lcg(&mut s) % span) as i16,
				lo + (lcg(&mut s) % span) as i16,
				lo + (lcg(&mut s) % span) as i16,
			);
			if lcg(&mut s) % 3 == 0 {
				assert_eq!(t.remove(&pos), oracle.remove(&pos), "remove ret @ {pos:?} op {op}");
			} else {
				let v = 1 + (lcg(&mut s) % 5) as u16;
				assert_eq!(t.insert(&pos, v), oracle.insert(pos, v), "insert ret @ {pos:?} op {op}");
			}
			if op % 97 == 0 {
				assert_matches_oracle(&t, &oracle);
			}
		}
		assert_matches_oracle(&t, &oracle);
	}

	#[test]
	fn model_tiny_range() { run_model(0, 4, 20_000, 1); }

	#[test]
	fn model_small_range() { run_model(0, 16, 50_000, 2); }

	#[test]
	fn model_with_negatives() { run_model(-16, 16, 50_000, 3); }

	#[test]
	fn model_subgrid_range() { run_model(0, 64, 200_000, 4); }

	#[test]
	fn model_depth3() { run_model(0, 200, 40_000, 11); }

	// Depth-4 trees, kept under the node-arena cap so no inserts are skipped.
	#[test]
	fn model_depth4() { run_model(0, 300, 6_000, 12); }

	#[test]
	fn model_sparse_wide() { run_model(-500, 500, 6_000, 5); }

	// Root depth must stay minimal for in-range (0..63) coords (no drift).
	#[test]
	fn root_depth_stays_bounded_for_local_coords() {
		let mut tree = GridTree::new();
		let mut s: u64 = 0x9e3779b9;
		for i in 0..200_000u64 {
			let pos = p(
				(lcg(&mut s) % 64) as i16,
				(lcg(&mut s) % 64) as i16,
				(lcg(&mut s) % 64) as i16,
			);
			if lcg(&mut s) % 3 == 0 { tree.remove(&pos); } else { tree.insert(&pos, 1); }
			assert!(
				tree.root_depth <= 3,
				"root_depth climbed to {} after {} ops (last pos {pos:?}, root_pos {:?})",
				tree.root_depth, i, tree.root_pos
			);
		}
	}
}

impl<'a> IntoIterator for &'a GridTree {
	type Item = (I16Vec3, u16, u16);
	type IntoIter = GridTreeIterator<'a>;

	fn into_iter(self) -> Self::IntoIter {
		self.iter()
	}
}

pub struct GridTreeIterator<'a> {
	tree: &'a GridTree,
	stack: Vec<(u32, u8, u8, I16Vec3)>,
}

impl<'a> GridTreeIterator<'a> {
	pub fn new(tree: &'a GridTree) -> Self {
		if tree.nodes.is_empty() {
			return Self {
				tree,
				stack: vec![]
			};
		}
		Self {
			tree,
			stack: vec![(0, 0, tree.root_depth, tree.root_pos)],
		}
	}
}

impl<'a> Iterator for GridTreeIterator<'a> {
	type Item = (I16Vec3, u16, u16);

	fn next(&mut self) -> Option<Self::Item> {
		loop {
			let (node_index, start_child, node_depth, node_origin) = self.stack.last_mut()?;
			let node_index = *node_index;
			let node_origin = *node_origin;
			let node_depth = *node_depth;

			let node = &self.tree.nodes[node_index as usize];
			let child_size = GridTreeNode::child_size(node_depth); // size of one cell in this node

			// Scan forward from where we left off
			let scan_start = *start_child;
			let mut found = false;

			for i in scan_start..SIZE_CUBED {
				let cell = node.contents[i as usize];
				let contents_pos = get_child_contents_pos(i);
				let child_world_origin = node_origin + (contents_pos.as_u16vec3() * child_size).as_i16vec3();

				match cell.value_type() {
					0 => { /* NONE – skip */ }
					1 => {
						// DATA leaf – yield it and resume after this cell next time
						*self.stack.last_mut().unwrap() = (node_index, i + 1, node_depth, node_origin);
						return Some((child_world_origin, child_size, cell.value()));
					}
					2 => {
						// NODE – push child onto stack, restart inner loop from there
						let child_node_index = node_index + cell.value() as u32;
						// Record that we should resume from i+1 when we pop back
						*self.stack.last_mut().unwrap() = (node_index, i + 1, node_depth, node_origin);
						self.stack.push((child_node_index, 0, node_depth - 1, child_world_origin));
						found = true;
						break;
					}
					_ => unsafe { unreachable_unchecked() }
				}
			}

			if !found {
				// Exhausted this node – pop it
				self.stack.pop();
			}
		}
	}
}
