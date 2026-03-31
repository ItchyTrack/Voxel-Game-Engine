use std::fmt::Debug;
use std::hint::unreachable_unchecked;
use std::u16;

use glam::{I8Vec3, I16Vec3, U8Vec3, U16Vec3, Vec3};


pub const LOG_SIZE: u8 = 2;
pub const SIZE: u8 = 1u8 << LOG_SIZE;
pub const SIZE_CUBED: u8 = SIZE * SIZE * SIZE;
pub const SIZE_USIZE: usize = SIZE as usize;
pub const SIZE_USIZE_CUBED: usize = SIZE_USIZE * SIZE_USIZE * SIZE_USIZE;

pub const MAX_TREE_DEPTH: u8 = 10; // it is lower than this but im being safe
pub const MAX_TREE_DEPTH_USIZE: usize = MAX_TREE_DEPTH as usize;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridTreeCell {
	// If value == 1<<16-1: NONE, Else if last bit is 0: DATA, Else last bit is 1: NODE.
	// NODE value != 1<<15-1 because then NONE
	pub value: u16,
}

impl GridTreeCell {
	const LAST_BIT_INDEX: u8 = u16::BITS as u8 - 1;
	pub const NONE: GridTreeCell = GridTreeCell{ value: u16::MAX };
	// You can only use the first 15 bits
	pub fn from_data(data: u16) -> Self {
		Self {
			value: data,
		}
	}
	// You can only use the first 15 bits and cant be 15 ones
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
	// 0: NONE, 1: DATA, 2: NODE
	pub fn value_type(&self) -> u8 {
		if self.value == u16::MAX { return 0; }
		1 + (self.value >> Self::LAST_BIT_INDEX) as u8
	}
	// Undefined output if NONE
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

#[derive(Debug)]
pub struct GridTreeNode {
	pub contents: [GridTreeCell; SIZE_USIZE_CUBED],
	pub parent_offset: u16, // if parent_index == 0 then no parent
	pub depth: u8, // 0 is the bottom
	pub used_cell_count: u8,
}

fn get_child_contents_index(contents_pos: U8Vec3) -> u8 {
	contents_pos.x + contents_pos.y * SIZE + contents_pos.z * SIZE * SIZE
}

fn get_child_contents_pos(contents_index: u8) -> U8Vec3 {
	U8Vec3::new(contents_index % SIZE, (contents_index / SIZE) % SIZE, contents_index / (SIZE * SIZE))
}

// fn get_relative_child_pos(child_pos: &I16Vec3, parent_pos: &I16Vec3, child_size: u16) -> U8Vec3 {
// 	((child_pos - parent_pos).as_u16vec3() / child_size).as_u8vec3()
// }

// fn get_parent_pos(child_pos: &I16Vec3, relative_child_pos: U8Vec3, child_size: u16) -> I16Vec3 {
// 	child_pos - (relative_child_pos.as_u16vec3() * child_size).as_i16vec3()
// }

impl GridTreeNode {
	fn new_root(depth: u8) -> Self {
		Self {
			contents: [GridTreeCell::NONE; SIZE_USIZE_CUBED],
			parent_offset: 0,
			depth: depth,
			used_cell_count: 0,
		}
	}
	fn new(parent_offset: u16, depth: u8) -> Self {
		Self {
			contents: [GridTreeCell::NONE; SIZE_USIZE_CUBED],
			parent_offset: parent_offset,
			depth: depth,
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
	fn size(&self) -> u16 {
		1 << (LOG_SIZE * (self.depth + 1))
	}
	fn child_size(&self) -> u16 {
		1 << (LOG_SIZE * self.depth)
	}
	fn parent_size(&self) -> u16 {
		1 << (LOG_SIZE * (self.depth + 2))
	}
	fn child_relative_pos(&self, child_contents_pos: U8Vec3) -> U16Vec3 {
		self.child_size() * child_contents_pos.as_u16vec3()
	}
	fn parent_relative_pos(&self, this_contents_pos: U8Vec3) -> U16Vec3 {
		self.size() * this_contents_pos.as_u16vec3()
	}
	// (type, value)
	fn get_child_cell_from_index(&self, contents_index: u8) -> (u8, u16) {
		let cell: GridTreeCell = self.contents[contents_index as usize];
		(cell.value_type(), cell.value())
	}
	// (type, value)
	fn get_child_cell(&self, contents_pos: U8Vec3) -> (u8, u16) {
		self.get_child_cell_from_index(get_child_contents_index(contents_pos))
	}
	fn get_child_cell_raw_from_index(&self, contents_index: u8) -> u16 {
		self.contents[contents_index as usize].value_raw()
	}
	fn get_child_cell_raw(&self, contents_pos: U8Vec3) -> u16 {
		self.get_child_cell_raw_from_index(get_child_contents_index(contents_pos))
	}
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

#[derive(Debug)]
pub struct GridTree {
	nodes: Vec<GridTreeNode>, // root at 0
	root_pos: I16Vec3,
	item_count: u64,
}

impl GridTree {
	pub fn new() -> Self {
		Self {
			nodes: vec![GridTreeNode::new_root(0)],
			root_pos: I16Vec3::ZERO,
			item_count: 0,
		}
	}
	fn add_child_node(&mut self, parent_index: u32, contents_pos: U8Vec3) -> u16 {
		let next_node_offset = (self.nodes.len() as u32 - parent_index) as u16;
		let parent = &mut self.nodes[parent_index as usize];
		assert!(next_node_offset != 0);
		parent.set_child_cell_to_node(contents_pos, next_node_offset);
		let parent_depth = parent.depth;
		self.nodes.push(GridTreeNode::new(next_node_offset, parent_depth - 1));
		next_node_offset
	}
	fn make_new_root(&mut self, contents_pos_of_old_root: U8Vec3) {
		let old_root = &mut self.nodes[0];
		old_root.set_parent_offset(Some(1));
		self.root_pos -= (old_root.size() * contents_pos_of_old_root.as_u16vec3()).as_i16vec3();
		let new_root_depth = old_root.depth + 1;
		self.nodes.insert(0, GridTreeNode {
			contents: {
				let mut contents = [GridTreeCell::NONE; SIZE_USIZE_CUBED];
				contents[get_child_contents_index(contents_pos_of_old_root) as usize] = GridTreeCell::from_node_offset(1);
				contents
			},
			parent_offset: 0,
			depth: new_root_depth,
			used_cell_count: 1,
		});
	}
	pub fn get(&self, pos: &I16Vec3) -> Option<u16> {
		let root_relative_pos = pos - self.root_pos;
		if root_relative_pos.is_negative_bitmask() != 0 { return None; }
		let root_relative_pos = root_relative_pos.as_u16vec3();
		let root = &self.nodes[0];
		if root_relative_pos.x >= root.size() || root_relative_pos.y >= root.size() || root_relative_pos.z >= root.size() { return None; }
		let mut current_node_index: u32 = 0;
		let mut current_relative_pos = root_relative_pos;
		loop {
			let node = &self.nodes[current_node_index as usize];
			let contents_pos = (current_relative_pos / node.child_size()).as_u8vec3();
			let cell = node.get_child_cell(contents_pos);
			match cell.0 {
				0 => return None,
				1 => return Some(cell.1),
				2 => {
					current_node_index += GridTreeCell::raw_as_node(cell.1) as u32;
					current_relative_pos %= node.child_size();
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

	fn make_sure_root_covers_pos(&mut self, pos: &I16Vec3) {
		let first_root = &self.nodes[0];
		if first_root.used_cell_count == 0 {
			self.root_pos = *pos;
			return;
		}
		let root_relative_pos = pos - self.root_pos;
		if root_relative_pos.is_negative_bitmask() != 0 ||
			root_relative_pos.x >= first_root.size() as i16 ||
			root_relative_pos.y >= first_root.size() as i16 ||
			root_relative_pos.z >= first_root.size() as i16
		{
			let mut min_pos = U8Vec3::MAX;
			let mut max_pos = U8Vec3::MIN;
			for contents_index in 0..SIZE_CUBED {
				if first_root.get_child_cell_from_index(contents_index).0 != 0 {
					let contents_pos = get_child_contents_pos(contents_index);
					min_pos = min_pos.min(contents_pos);
					max_pos = max_pos.max(contents_pos);
				}
			}
			if max_pos != U8Vec3::splat(SIZE - 1) && min_pos != U8Vec3::ZERO {
				let root_relative_contents_pos = root_relative_pos.div_euclid(I16Vec3::splat(first_root.child_size() as i16));
				let root_shift_amount = I16Vec3::new(
					if root_relative_contents_pos.x > 0 { (root_relative_contents_pos.x + 1 - SIZE as i16).clamp(0, SIZE as i16 - 1) } else { root_relative_contents_pos.x.max(1 - SIZE as i16) },
					if root_relative_contents_pos.y > 0 { (root_relative_contents_pos.y + 1 - SIZE as i16).clamp(0, SIZE as i16 - 1) } else { root_relative_contents_pos.y.max(1 - SIZE as i16) },
					if root_relative_contents_pos.z > 0 { (root_relative_contents_pos.z + 1 - SIZE as i16).clamp(0, SIZE as i16 - 1) } else { root_relative_contents_pos.z.max(1 - SIZE as i16) },
				).clamp((SIZE - 1 - max_pos).as_i16vec3(), min_pos.as_i16vec3());
				if root_shift_amount != I16Vec3::ZERO {
					let mut shifted_root_contents = [GridTreeCell::NONE; SIZE_USIZE_CUBED];
					for x in min_pos.x..=max_pos.x {
						for y in min_pos.y..=max_pos.y {
							for z in min_pos.z..=max_pos.z {
								let contents_pos = U8Vec3::new(x, y, z);
								let (cell_type, cell_raw) = first_root.get_child_cell_type_and_raw(contents_pos);
								if cell_type != 0 {
									shifted_root_contents[get_child_contents_index((contents_pos.as_i16vec3() - root_shift_amount).as_u8vec3()) as usize] = GridTreeCell::from_raw(cell_raw);
								}
							}
						}
					}
					self.root_pos += root_shift_amount * first_root.child_size() as i16;
					self.nodes[0].contents = shifted_root_contents;
				}
			}
		} else {
			return;
		}
		loop {
			let root_relative_pos = pos - self.root_pos;
			let root = &self.nodes[0];
			if root_relative_pos.is_negative_bitmask() != 0 ||
				root_relative_pos.x >= root.size() as i16 ||
				root_relative_pos.y >= root.size() as i16 ||
				root_relative_pos.z >= root.size() as i16
			{
				let contents_pos_of_old_root = U8Vec3::new(
					if root_relative_pos.x < 0 { SIZE - 1 } else { 0 },
					if root_relative_pos.y < 0 { SIZE - 1 } else { 0 },
					if root_relative_pos.z < 0 { SIZE - 1 } else { 0 },
				);
				self.make_new_root(contents_pos_of_old_root);
			} else {
				break;
			}
		}
	}
	// parent depth must be more than 0 and at pos in parent there must be a data cell containing current_cell and current_cell != cell_to_set
	fn set_voxel_in_data_cell(&mut self, parent_node_index: u32, current_cell: GridTreeCell, cell_to_set: GridTreeCell, pos: &U16Vec3) {
		let next_node_offset = (self.nodes.len() as u32 - parent_node_index) as u16;
		let parent = &mut self.nodes[parent_node_index as usize];
		let child_size = parent.child_size();
		let relative_pos = (pos / child_size).as_u8vec3();
		assert!(next_node_offset != 0);
		parent.set_child_cell_to_node(relative_pos, next_node_offset);
		let parent_depth = parent.depth;
		self.nodes.push(GridTreeNode {
			contents: [current_cell; SIZE_USIZE_CUBED],
			parent_offset: next_node_offset,
			depth : parent_depth - 1,
			used_cell_count: SIZE_CUBED,
		});
		if parent_depth == 1 {
			let node = &mut self.nodes[(parent_node_index + next_node_offset as u32) as usize];
			if cell_to_set.value_type() == 0 {
				node.used_cell_count -= 1;
			}
			node.set_child_cell((pos % SIZE as u16).as_u8vec3(), cell_to_set);
		} else {
			self.set_voxel_in_data_cell(parent_node_index + next_node_offset as u32, current_cell, cell_to_set, &(pos % child_size));
		}
	}
	// parent depth must be more than 0 and at pos in parent there must be a none cell and cell_to_set must be type 1
	fn set_voxel_in_none_cell(&mut self, parent_node_index: u32, cell_to_set: GridTreeCell, pos: &U16Vec3) {
		assert!(cell_to_set.value_type() == 1);
		let next_node_offset = (self.nodes.len() as u32 - parent_node_index) as u16;
		let parent = &mut self.nodes[parent_node_index as usize];
		parent.used_cell_count += 1;
		let child_size = parent.child_size();
		let contents_pos = (pos / child_size).as_u8vec3();
		assert!(next_node_offset != 0);
		parent.set_child_cell_to_node(contents_pos, next_node_offset);
		let parent_depth = parent.depth;
		self.nodes.push(GridTreeNode {
			contents: [GridTreeCell::NONE; SIZE_USIZE_CUBED],
			parent_offset: next_node_offset,
			depth : parent_depth - 1,
			used_cell_count: 1,
		});
		if parent_depth == 1 {
			let node = &mut self.nodes[(parent_node_index + next_node_offset as u32) as usize];
			node.set_child_cell((pos % SIZE as u16).as_u8vec3(), cell_to_set);
		} else {
			self.set_voxel_in_none_cell(parent_node_index + next_node_offset as u32, cell_to_set, &(pos % child_size));
		}
	}
	pub fn get_internals(&self) -> (&Vec<GridTreeNode>, I16Vec3) {
        (&self.nodes, self.root_pos)
    }
	// cell_to_merge cant be NODE. pos_in_node is any pos
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
	// Assumes childern are dead. // Does nothing (leaks the memory)
	pub fn remove_node(&mut self, _node_index: u32) {

	}

	pub fn insert(&mut self, pos: &I16Vec3, data: u16) -> Option<u16> {
		self.make_sure_root_covers_pos(pos);
		let mut current_node_index: u32 = 0;
		let mut current_relative_pos = (pos - self.root_pos).as_u16vec3();
		let mut cell_index_stack = [0; MAX_TREE_DEPTH_USIZE];
		let mut cell_index_stack_size = 0;
		loop {
			let node = &mut self.nodes[current_node_index as usize];
			let contents_pos = (current_relative_pos / node.child_size()).as_u8vec3();
			let contents_index = get_child_contents_index(contents_pos);
			cell_index_stack[cell_index_stack_size] = contents_index;
			let cell = node.get_child_cell_type_and_raw_from_index(contents_index);
			match cell.0 {
				0 => {
					if node.depth == 0 {
						node.used_cell_count += 1;
						node.set_child_cell_to_data(contents_pos, data);
						self.try_merge(current_node_index, data, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_none_cell(current_node_index, GridTreeCell::from_data(data), &current_relative_pos);
					}
					self.item_count += 1;
					return None;
				},
				1 => {
					if GridTreeCell::raw_as_data(cell.1) == data {
						return Some(GridTreeCell::raw_as_data(cell.1));
					}
					if node.depth == 0 {
						node.set_child_cell_to_data(contents_pos, data);
						self.try_merge(current_node_index, data, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_data_cell(current_node_index, GridTreeCell::from_raw(cell.1), GridTreeCell::from_data(data), &current_relative_pos);
					}
					return Some(GridTreeCell::raw_as_data(cell.1));
				},
				2 => {
					current_node_index += GridTreeCell::raw_as_node(cell.1) as u32;
					current_relative_pos %= node.child_size();
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
		let root = &self.nodes[0];
		if root_relative_pos.x >= root.size() || root_relative_pos.y >= root.size() || root_relative_pos.z >= root.size() { return None; }
		let mut current_node_index: u32 = 0;
		let mut current_relative_pos = root_relative_pos;
		let mut cell_index_stack = [0; MAX_TREE_DEPTH_USIZE];
		let mut cell_index_stack_size = 0;
		loop {
			let node = &mut self.nodes[current_node_index as usize];
			let contents_pos = (current_relative_pos / node.child_size()).as_u8vec3();
			let contents_index = get_child_contents_index(contents_pos);
			cell_index_stack[cell_index_stack_size] = contents_index;
			let cell = node.get_child_cell_type_and_raw_from_index(contents_index);
			match cell.0 {
				0 => return None,
				1 => {
					if node.depth == 0 {
						node.set_child_cell_to_none(contents_pos);
						node.used_cell_count -= 1;
						self.try_merge_empty(current_node_index, &cell_index_stack[0..cell_index_stack_size]);
					} else {
						self.set_voxel_in_data_cell(current_node_index, GridTreeCell::from_raw(cell.1), GridTreeCell::NONE, &current_relative_pos);
					}
					self.item_count -= 1;
					return Some(cell.1);
				},
				2 => {
					current_node_index += GridTreeCell::raw_as_node(cell.1) as u32;
					current_relative_pos %= node.child_size();
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

	pub fn raycast(&self, pose: &Pose, max_length: Option<f32>) -> Option<(I16Vec3, I8Vec3, f32)> {
		let max_length = max_length.unwrap_or(f32::MAX);

		let origin = pose.translation;
		let dir = pose.rotation * Vec3::Z;

		// debug_draw::line(debug_pose * (origin), debug_pose * (origin + dir * 30.0), &Vec4::new(0.0, 0.0, 0.0, 1.0));

		let root = &self.nodes[0];
		let root_min = self.root_pos.as_vec3();
		let root_max = root_min + Vec3::splat(root.size() as f32);

		// Ray vs root AABB
		let distance_to_aabb = match Self::ray_aabb_intersection(&origin, &dir, &(root_min, root_max)) {
			Some(dis) => dis,
			None => return None,
		};
		let post_aabb_origin_pre_shift = origin + dir * distance_to_aabb;
		let post_aabb_origin = post_aabb_origin_pre_shift
			.min(self.root_pos.as_vec3() + Vec3::splat(((root.size()) as f32) - 0.00001))
			.max(self.root_pos.as_vec3());
		let post_aabb_origin = post_aabb_origin.move_towards(post_aabb_origin.floor() + 0.5, 0.001);
		let root_relative_post_aabb_origin = post_aabb_origin - self.root_pos.as_vec3();
		let delta = dir.recip().abs();
		let step = dir.signum().as_i8vec3();
		let mut axis_distances = dir.recip() * Vec3::new(
			if step.x > 0 { root_relative_post_aabb_origin.x.ceil() } else { root_relative_post_aabb_origin.x.floor() } - root_relative_post_aabb_origin.x,
			if step.y > 0 { root_relative_post_aabb_origin.y.ceil() } else { root_relative_post_aabb_origin.y.floor() } - root_relative_post_aabb_origin.y,
			if step.z > 0 { root_relative_post_aabb_origin.z.ceil() } else { root_relative_post_aabb_origin.z.floor() } - root_relative_post_aabb_origin.z,
		);
		// debug_draw::line(debug_pose * (origin + Vec3::new(0.0, 0.025, 0.0)), debug_pose * (origin + Vec3::new(0.0, 0.025, 0.0) + dir * axis_distances.x), &Vec4::new(1.0, 0.2, 0.2, 1.0));
		// debug_draw::line(debug_pose * (origin + Vec3::new(0.0, 0.05, 0.0)), debug_pose * (origin + Vec3::new(0.0, 0.05, 0.0) + dir * axis_distances.y), &Vec4::new(0.2, 1.0, 0.2, 1.0));
		// debug_draw::line(debug_pose * (origin + Vec3::new(0.0, 0.075, 0.0)), debug_pose * (origin + Vec3::new(0.0, 0.075, 0.0) + dir * axis_distances.z), &Vec4::new(0.2, 0.2, 1.0, 1.0));
		// debug_draw::point(debug_pose * (root_relative_post_aabb_origin + self.root_pos.as_vec3()), &Vec4::W, 1.0);
		let mut root_relative_grid_pos = root_relative_post_aabb_origin.as_u16vec3();
		let mut last_step_axis = (post_aabb_origin_pre_shift - post_aabb_origin).abs().max_position() as u8;
		let mut current_node_index = 0u32;
		let mut last_distance = distance_to_aabb;
		if max_length < last_distance { return None; }
		loop {
			let mut current_node = &self.nodes[current_node_index as usize];
			// debug_draw::rectangular_prism(&(debug_pose * Pose::from_translation((self.root_pos + root_relative_grid_pos.as_i16vec3()).as_vec3())), Vec3::ONE, &Vec4::ONE, false);
			let node_relative_grid_pos = root_relative_grid_pos % current_node.size();
			// debug_draw::rectangular_prism(&(debug_pose * Pose::from_translation((
			// 	self.root_pos + root_relative_grid_pos.as_i16vec3() - node_relative_grid_pos.as_i16vec3()
			// ).as_vec3())), Vec3::splat(current_node.size() as f32), &Vec4::new(0.0, 0.0, 1.0, 1.0), false);
			let contents_pos = (node_relative_grid_pos / current_node.child_size()).as_u8vec3();
			let cell = current_node.get_child_cell(contents_pos);
			match cell.0 {
				0 => { // NONE
					if current_node.child_size() != 1 {
						let node_cell_relative_grid_pos = node_relative_grid_pos % current_node.child_size();
						let mut step_amount = U16Vec3::select(
							step.cmpgt(I8Vec3::ZERO),
							U16Vec3::splat(current_node.child_size() - 1) - node_cell_relative_grid_pos,
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
						// debug_draw::rectangular_prism(&(debug_pose * Pose::from_translation((self.root_pos + root_relative_grid_pos.as_i16vec3()).as_vec3())), Vec3::ONE, &Vec4::W, false);
						// println!("{root_relative_grid_pos}");
					}
					match axis_distances.min_position() {
						0 => {
							if max_length < axis_distances.x { return None; }
							let root_relative_grid_pos_x = root_relative_grid_pos.x as i16 + step.x as i16;
							if root_relative_grid_pos_x < 0 || root_relative_grid_pos_x >= root.size() as i16 { return None; }
							let root_relative_grid_pos_x = root_relative_grid_pos_x as u16;
							loop {
								if root_relative_grid_pos.x / current_node.size() == root_relative_grid_pos_x / current_node.size() {
									break;
								}
								current_node_index -= current_node.get_parent_offset().unwrap() as u32;
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
							if root_relative_grid_pos_y < 0 || root_relative_grid_pos_y >= root.size() as i16 { return None; }
							let root_relative_grid_pos_y = root_relative_grid_pos_y as u16;
							loop {
								if root_relative_grid_pos.y / current_node.size() == root_relative_grid_pos_y / current_node.size() {
									break;
								}
								current_node_index -= current_node.get_parent_offset().unwrap() as u32;
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
							if root_relative_grid_pos_z < 0 || root_relative_grid_pos_z >= root.size() as i16 { return None; }
							let root_relative_grid_pos_z = root_relative_grid_pos_z as u16;
							loop {
								if root_relative_grid_pos.z / current_node.size() == root_relative_grid_pos_z / current_node.size() {
									break;
								}
								current_node_index -= current_node.get_parent_offset().unwrap() as u32;
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
					current_node_index += cell.1 as u32;
				}
				_ => unsafe { unreachable_unchecked(); }
			}
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
	stack: Vec<(u32, u8, I16Vec3)>,
}

impl<'a> GridTreeIterator<'a> {
	pub fn new(tree: &'a GridTree) -> Self {
		if tree.nodes.is_empty() {
			return Self { tree, stack: vec![] };
		}
		Self {
			tree,
			stack: vec![(0, 0, tree.root_pos)],
		}
	}
}

impl<'a> Iterator for GridTreeIterator<'a> {
	type Item = (I16Vec3, u16, u16);

	fn next(&mut self) -> Option<Self::Item> {
		loop {
			let (node_index, start_child, node_origin) = self.stack.last_mut()?;
			let node_index = *node_index;
			let node_origin = *node_origin;

			let node = &self.tree.nodes[node_index as usize];
			let child_size = node.child_size(); // size of one cell in this node

			// Scan forward from where we left off
			let scan_start = *start_child;
			let mut found = false;

			for i in scan_start..SIZE_CUBED {
				let cell = node.contents[i as usize];
				let contents_pos = get_child_contents_pos(i);
				let child_world_origin = node_origin
					+ (contents_pos.as_u16vec3() * child_size).as_i16vec3();

				match cell.value_type() {
					0 => { /* NONE – skip */ }
					1 => {
						// DATA leaf – yield it and resume after this cell next time
						*self.stack.last_mut().unwrap() =
							(node_index, i + 1, node_origin);
						return Some((child_world_origin, child_size, cell.value()));
					}
					2 => {
						// NODE – push child onto stack, restart inner loop from there
						let child_node_index = node_index + cell.value() as u32;
						// Record that we should resume from i+1 when we pop back
						*self.stack.last_mut().unwrap() =
							(node_index, i + 1, node_origin);
						self.stack.push((child_node_index, 0, child_world_origin));
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
