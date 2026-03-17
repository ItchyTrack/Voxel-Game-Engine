use std::fmt::Debug;
use std::mem::replace;

use glam::{I8Vec3, I16Vec3, Quat, U8Vec3, U16Vec3, Vec3, Vec4};

use crate::debug_draw;
use crate::pose::Pose;

const SIZE: u8 = 2;
const SIZE_USIZE: usize = SIZE as usize;
const SIZE_USIZE_CUBED: usize = SIZE_USIZE * SIZE_USIZE * SIZE_USIZE;

#[derive(Clone, Copy, Debug)]
pub enum ChildCell<T: Copy + Clone + PartialEq + Debug> {
	None,
	Node { node: u32 },
	Data { data: T },
}

#[derive(Debug)]
struct GridTreeNode<T: Copy + Clone + PartialEq + Debug> {
	contents: [ChildCell<T>; SIZE_USIZE_CUBED],
	child_count: u8,
	parent: Option<u32>,
	pos: I16Vec3,
	scale: u16,
}

fn get_child_index(pos: &U8Vec3) -> u8 {
	pos.x + pos.y * SIZE + pos.z * SIZE * SIZE
}

fn get_relitive_child_pos(child_pos: &I16Vec3, parent_pos: &I16Vec3, child_size: u16) -> U8Vec3 {
	((child_pos - parent_pos).as_u16vec3() / child_size).as_u8vec3()
}

fn get_parent_pos(child_pos: &I16Vec3, relitive_child_pos: U8Vec3, child_size: u16) -> I16Vec3 {
	child_pos - (relitive_child_pos.as_u16vec3() * child_size).as_i16vec3()
}

impl<T: Copy + Clone + PartialEq + Debug> GridTreeNode<T> {
	fn new(parent: Option<u32>, pos: I16Vec3, scale: u16) -> Self {
		Self {
			contents: [ChildCell::None; SIZE_USIZE_CUBED],
			child_count: 0,
			parent: parent,
			pos: pos,
			scale: scale,
		}
	}
	fn get_child_cell_pos(&self, index: u8) -> U8Vec3 {
		U8Vec3::new(index % SIZE, (index / SIZE) % SIZE, index / (SIZE * SIZE))
	}
	fn get_child_cell(&self, pos: &U8Vec3) -> &ChildCell<T> {
		&self.contents[get_child_index(pos) as usize]
	}
	fn get_child_cell_mut(&mut self, pos: &U8Vec3) -> &mut ChildCell<T> {
		&mut self.contents[get_child_index(pos) as usize]
	}
	fn get_child_cell_world_pos(&self, index: u8) -> I16Vec3 {
		(self.get_child_cell_pos(index).as_u16vec3() * self.child_wold_size()).as_i16vec3() + self.pos
	}
	fn child_wold_size(&self) -> u16 {
		self.scale
	}
	fn world_size(&self) -> u16 {
		self.scale * SIZE as u16
	}
}

#[derive(Debug)]
pub struct GridTree<T: Copy + Clone + PartialEq + Debug> {
	nodes: Vec<GridTreeNode<T>>,
	root: u32,
	item_count: u64,
}

impl<T: Copy + Clone + PartialEq + Debug> GridTree<T> {
	pub fn new() -> Self {
		Self {
			nodes: vec![GridTreeNode::new(None, I16Vec3::splat(SIZE as i16 / -2 + 1), 1)],
			root: 0,
			item_count: 0,
		}
	}
	fn add_node(&mut self, parent: Option<u32>, pos: I16Vec3, scale: u16) -> u32 {
		assert!(self.get_next_added_node_index() == self.nodes.len() as u32);
		self.nodes.push(GridTreeNode::new(parent, pos, scale));
		self.nodes.len() as u32 - 1
	}
	fn get_next_added_node_index(&self) -> u32 {
		self.nodes.len() as u32
	}
	fn get_node(&self, index: u32) -> &GridTreeNode<T> {
		self.nodes.get(index as usize).unwrap()
	}
	fn get_node_mut(&mut self, index: u32) -> &mut GridTreeNode<T> {
		self.nodes.get_mut(index as usize).unwrap()
	}
	pub fn remove(&mut self, pos: &I16Vec3) -> Option<T> {
		let root = self.get_node(self.root);
		let root_relative_pos = pos - root.pos;
		if root_relative_pos.is_negative_bitmask() != 0 { return None; }
		let root_size = root.world_size();
		let mut upos = root_relative_pos.as_u16vec3();
		if upos.x >= root_size || upos.y >= root_size || upos.z >= root_size { return None; }
		let mut index = self.root;
		loop {
			let node = self.get_node_mut(index);
			let node_size = node.scale;
			let local_pos = (upos / node_size).as_u8vec3();
			upos %= node_size;
			let cell = node.get_child_cell_mut(&local_pos);
			match cell {
				ChildCell::None => { return None; },
				ChildCell::Node { node: child_node } => {
					index = *child_node;
				},
				ChildCell::Data { data: _ } => {
					let out = match replace(cell, ChildCell::None) {
						ChildCell::Data { data } => data,
						_ => unreachable!()
					};
					node.child_count -= 1;
					self.item_count -= 1;
					return Some(out);
				},
			};
		}
	}
	pub fn get_cell(&self, pos: &I16Vec3) -> Option<&ChildCell<T>> {
		let root = self.get_node(self.root);
		let root_relative_pos = pos - root.pos;
		if root_relative_pos.is_negative_bitmask() != 0 { return None; }
		let root_size = root.scale * SIZE as u16;
		let mut upos = root_relative_pos.as_u16vec3();
		if upos.x >= root_size || upos.y >= root_size || upos.z >= root_size { return None; }
		let mut index = self.root;
		loop {
			let node = self.get_node(index);
			let node_size = node.scale;
			let local_pos = (upos / node_size).as_u8vec3();
			upos %= node_size;
			let cell = node.get_child_cell(&local_pos);
			match cell {
				ChildCell::Node { node: child_node } => {
					index = *child_node;
				}
				_ => return Some(cell),
			};
		}
	}
	pub fn get(&self, pos: &I16Vec3) -> Option<&T> {
		match self.get_cell(&pos) {
			Some( value ) => match value {
				crate::grid_tree::ChildCell::Data { data } => return Some(data),
				_ => None,
			},
			None => None,
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
	// Assumes childern are dead. Returns any mapping update.
	pub fn remove_node(&mut self, node_index: u32) -> Option<(u32, u32)> {
		if node_index as usize + 1 == self.nodes.len() {
			self.nodes.pop();
			return None;
		}
		let end = self.nodes.pop().unwrap();
		for cell in end.contents {
			match cell {
				ChildCell::Node { node: child_index } => {
					*self.nodes[child_index as usize].parent.as_mut().unwrap() = node_index;
				},
				_ => { }
			}
		}
		if let Some(parent_index) = end.parent {
			if (parent_index as usize) < self.nodes.len() {
				let end_pos = end.pos;
				let parent = &mut self.nodes[parent_index as usize];
				let relitive_pos = get_relitive_child_pos(&end_pos, &parent.pos, parent.scale);
				parent.contents[get_child_index(&relitive_pos) as usize] = ChildCell::Node { node: node_index };
			}
		}
		self.nodes[node_index as usize] = end;
		if (self.root as usize) == self.nodes.len() {
			self.root = node_index;
		}
		return Some((self.nodes.len() as u32, node_index));
	}
	pub fn try_merge(&mut self, node_index: u32, value: T, ignored_index: u8) -> bool {
		let node = self.get_node(node_index);
		if node.child_count < 7 {
			return false;
		}
		for child_index in 0..8 {
			if child_index == ignored_index { continue; }
			match node.contents[child_index as usize] {
				ChildCell::Data { data } => if data != value { return false },
				_ => return false,
			}
		}
		let child_pos = node.pos;
		if let Some(mut parent_index) = node.parent {
			if let Some((pre, post)) = self.remove_node(node_index) {
				if parent_index == pre {
					parent_index = post;
				}
			}
			let parent = self.get_node_mut(parent_index);
			let local_child_pos = get_relitive_child_pos(&child_pos, &parent.pos, parent.scale);
			if self.try_merge(parent_index, value, get_child_index(&local_child_pos)) {
				return true;
			}
			*self.get_node_mut(parent_index).get_child_cell_mut(&local_child_pos) = ChildCell::Data { data: value };
			return true;
		}
		return false;
	}
	pub fn insert(&mut self, pos: I16Vec3, t: T) -> Option<T> {
		let root = self.get_node(self.root);
		let mut root_world_pos = root.pos;
		let mut root_scale = root.scale;
		let mut root_relative_pos = pos - root_world_pos;
		let mut root_size = root_scale * SIZE as u16;
		let mut upos = root_relative_pos.as_u16vec3();
		while root_relative_pos.is_negative_bitmask() != 0 || upos.x >= root_size || upos.y >= root_size || upos.z >= root_size {
			let old_root = self.root;
			let new_root_pos = SIZE / 2 - (root_relative_pos.is_negative_bitmask() == 0) as u8;
			root_world_pos = root_world_pos - I16Vec3::splat((root_size * new_root_pos as u16) as i16);
			self.root = self.add_node(None, root_world_pos, root_scale * SIZE as u16);
			self.get_node_mut(old_root).parent = Some(self.root);
			let new_root_node = self.get_node_mut(self.root);
			new_root_node.child_count += 1;
			*new_root_node.get_child_cell_mut(&U8Vec3::splat(new_root_pos)) = ChildCell::Node { node: old_root };
			root_relative_pos = pos - root_world_pos;
			upos = root_relative_pos.as_u16vec3();
			root_scale = root_size;
			root_size *= SIZE as u16;
		}
		let mut index = self.root;
		loop {
			let node = self.get_node(index);
			let node_scale = node.scale;
			let local_pos = (upos / node_scale).as_u8vec3();
			if node.scale == 1 {
				let local_pos_index = get_child_index(&local_pos);
				let cell = &node.contents[local_pos_index as usize];
				match cell {
					ChildCell::None => {
						if self.try_merge(index, t, local_pos_index) {
							return None;
						}
						let node = self.get_node_mut(index);
						let cell = &mut node.contents[local_pos_index as usize];
						*cell = ChildCell::Data { data: t };
						node.child_count += 1;
						self.item_count += 1;
						return None;
					},
					ChildCell::Node { node: _ } => unreachable!(),
					ChildCell::Data { data } => {
						let held_data = data.clone();
						if self.try_merge(index, t, local_pos_index) {
							return Some(held_data);
						}
						let node = self.get_node_mut(index);
						node.contents[local_pos_index as usize] = ChildCell::Data { data: t };
						return Some(held_data);
					},
				}
			}
			upos %= node_scale;
			let node_pos = node.pos;
			let node = self.get_node_mut(index);
			let cell = node.get_child_cell_mut(&local_pos);
			match cell {
				ChildCell::None => {
					unsafe {
						let ptr = cell as *mut ChildCell<T>;
						node.child_count += 1;
						*ptr = ChildCell::Node { node: self.get_next_added_node_index() };
					};
					index = self.add_node(Some(index), node_pos + (node_scale * local_pos.as_u16vec3()).as_i16vec3(), node_scale / SIZE as u16);
				},
				ChildCell::Node { node: child_node } => {
					index = *child_node;
				}
				ChildCell::Data { data: _ } => { // TODO: fill rest of node with data.
					unsafe {
						let ptr = cell as *mut ChildCell<T>;
						node.child_count += 1;
						*ptr = ChildCell::Node { node: self.get_next_added_node_index() };
					};
					index = self.add_node(Some(index), node_pos + (node_scale * local_pos.as_u16vec3()).as_i16vec3(), node_scale / SIZE as u16);
				}
			};
		}
	}
	pub fn len(&self) -> u64 { return self.item_count; }
	pub fn is_empty(&self) -> bool { return self.item_count == 0; }
	pub fn iter<'a>(&'a self) -> GridTreeIterator<'a, T> {
		GridTreeIterator::<'a>::new(self)
	}
	pub fn raycast(&self, pose: &Pose, max_length: Option<f32>) -> Option<(I16Vec3, I8Vec3, f32)> {
		let max_length = max_length.unwrap_or(f32::MAX);
		let origin = pose.translation;
		let dir = pose.rotation * Vec3::Z;
		let inv_dir = Vec3::ONE / dir;
		let dir_sign = I16Vec3::new(
			(dir.x >= 0.0) as i16,
			(dir.y >= 0.0) as i16,
			(dir.z >= 0.0) as i16,
		);
		self.dda_node(self.root, origin, inv_dir, dir_sign, 0.0, max_length)
	}

	fn dda_node<'a>(
		&'a self,
		node_idx: u32,
		origin: Vec3,
		inv_dir: Vec3,
		dir_sign: I16Vec3,
		t_min: f32,
		t_max: f32,
	) -> Option<(I16Vec3, I8Vec3, f32)> {
		let node = &self.nodes[node_idx as usize];
		let cell_size = node.scale as f32;
		let node_pos = node.pos.as_vec3();
		let step = dir_sign * 2 - I16Vec3::ONE;

		let entry_world = (origin + inv_dir.recip() * t_min).clamp(
			node_pos,
			node_pos + Vec3::splat(node.world_size() as f32 - f32::EPSILON * cell_size),
		);
		let mut cell = ((entry_world - node_pos) / cell_size)
			.floor()
			.as_i16vec3()
			.clamp(I16Vec3::ZERO, I16Vec3::splat(SIZE as i16 - 1));

		let cell_exit_face = node_pos + (cell + dir_sign).as_vec3() * cell_size;
		let mut t_next = (cell_exit_face - origin) * inv_dir;
		let t_delta = (Vec3::splat(cell_size) * inv_dir).abs();

		let mut t_current = t_min;

		let t_entry = t_next - t_delta;
		let mut normal_axis = if t_entry.x > t_entry.y && t_entry.x > t_entry.z { 0 } else if t_entry.y > t_entry.z { 1 } else { 2 };

		loop {
			if cell.x < 0 || cell.x >= SIZE as i16
			|| cell.y < 0 || cell.y >= SIZE as i16
			|| cell.z < 0 || cell.z >= SIZE as i16 {
				return None;
			}

			let t_cell_exit = t_next.min_element().min(t_max);
			let idx = get_child_index(&cell.as_u8vec3());

			match &node.contents[idx as usize] {
				ChildCell::None => {}
				ChildCell::Data { data: _ } => {
					if t_current <= t_max {
						let hit_world = origin + inv_dir.recip() * t_current;
						let mut normal = I8Vec3::ZERO;
						normal[normal_axis] = -(step[normal_axis] as i8);
						let scale1_pos = (hit_world - normal.as_vec3() * 0.5).floor().as_i16vec3();
						return Some((scale1_pos, normal, t_current));
					} else {
						return None;
					}
				}
				ChildCell::Node { node: child_idx } => {
					if let Some(hit) = self.dda_node(*child_idx, origin, inv_dir, dir_sign, t_current, t_cell_exit) {
						return Some(hit);
					}
				}
			}

			if t_next.x < t_next.y {
				if t_next.x < t_next.z {
					if t_next.x > t_max { return None; }
					t_current = t_next.x; normal_axis = 0;
					cell.x += step.x; t_next.x += t_delta.x;
				} else {
					if t_next.z > t_max { return None; }
					t_current = t_next.z; normal_axis = 2;
					cell.z += step.z; t_next.z += t_delta.z;
				}
			} else if t_next.y < t_next.z {
				if t_next.y > t_max { return None; }
				t_current = t_next.y; normal_axis = 1;
				cell.y += step.y; t_next.y += t_delta.y;
			} else {
				if t_next.z > t_max { return None; }
				t_current = t_next.z; normal_axis = 2;
				cell.z += step.z; t_next.z += t_delta.z;
			}
		}
	}

	fn node_entry_t(&self, origin: Vec3, inv_dir: Vec3, aabb: &(Vec3, Vec3)) -> f32 {
		let t1 = (aabb.0 - origin) * inv_dir;
		let t2 = (aabb.1 - origin) * inv_dir;
		t1.min(t2).max_element()
	}
	pub fn render_debug(&self, pose: &Pose) {
		let mut stack = vec![self.root];
		while let Some(idx) = stack.pop() {
			let node = &self.nodes[idx as usize];
			let node_size = node.scale * SIZE as u16;
			debug_draw::rectangular_prism(&(pose * Pose::new(node.pos.as_vec3(), Quat::IDENTITY)), Vec3::splat(node_size as f32), &Vec4::new(1.0, 1.0, 1.0, 0.0075), true);
			if node.child_count == 0 {
				continue;
			} else {
				for i in 0..SIZE.pow(3) as u8 {
					match &node.contents[i as usize] {
						ChildCell::Node { node: child_node } => {
							stack.push(*child_node);
						},
						ChildCell::Data { data: _ } => { },
						ChildCell::None => { },
					};
				}
			}
		}
	}
	pub fn print_debug_rec(&self, index: u32) {
		if self.nodes.len() <= index as usize {
			println!("out of range node as {}!", index);
			return;
		}
		let node = &self.nodes[index as usize];
		if node.scale == 1 {
			return;
		}
		for _ in 0..(20 - ((node.scale) as f32).log2() as u32) {
			print!(" ");
		}
		let mut any = false;
		println!("{} [", index);
		for child in node.contents {
			match child {
				ChildCell::None => { },
				ChildCell::Node { node } => {
					any = true;
					self.print_debug_rec(node);
				},
				ChildCell::Data { data: _ } => { },
			}
		}
		if any {
			for _ in 0..(20 - ((node.scale) as f32).log2() as u32) {
				print!(" ");
			}
			println!("]");
		}
	}
	pub fn print_debug(&self) {
		self.print_debug_rec(self.root);
	}
}

pub struct GridTreeIterator<'a, T: Copy + Clone + PartialEq + Debug> {
	tree: &'a GridTree<T>,
	current_node: u32,
	current_index: u8,
}

impl<'a, T: Copy + Clone + PartialEq + Debug> GridTreeIterator<'a, T> {
	pub fn new(tree: &'a GridTree<T>) -> Self {
		Self {
			tree: tree,
			current_node: tree.root,
			current_index: 0,
		}
	}
}

impl<'a, T: Copy + Clone + PartialEq + Debug> Iterator for GridTreeIterator<'a, T> {
	type Item = (I16Vec3, u16, &'a T);

	fn next(&mut self) -> Option<Self::Item> {
		let mut node: &GridTreeNode<T> = self.tree.get_node(self.current_node);
		loop {
			if node.child_count == 0 {
				if node.parent.is_none() { return None; }
				let pos = node.pos;
				self.current_node = node.parent.unwrap();
				node = self.tree.get_node(self.current_node);
				self.current_index = get_child_index(&((pos - node.pos).as_u16vec3() / node.scale).as_u8vec3()) + 1;
			} else {
				for i in self.current_index..SIZE.pow(3) {
					match &node.contents[i as usize] {
						ChildCell::Node { node: child_node } => {
							self.current_index = 0;
							self.current_node = *child_node;
							node = self.tree.get_node(self.current_node);
							break;
						},
						ChildCell::Data { data } => {
							self.current_index = i + 1;
							return Some((
								node.get_child_cell_world_pos(i),
								node.scale,
								data
							));
						},
						ChildCell::None => { },
					};
				}
				if self.current_index != 0 {
					if node.parent.is_none() { return None; }
					let pos = node.pos;
					self.current_node = node.parent.unwrap();
					node = self.tree.get_node(self.current_node);
					self.current_index = get_child_index(&((pos - node.pos).as_u16vec3() / node.scale).as_u8vec3()) + 1;
				}
			}
		}
	}
}

impl<'a, T: Copy + Clone + PartialEq + Debug> IntoIterator for &'a GridTree<T> {
	type Item = (I16Vec3, u16, &'a T);
	type IntoIter = GridTreeIterator<'a, T>;

	fn into_iter(self) -> Self::IntoIter {
		self.iter()
	}
}

struct DDAState {
	pos: I16Vec3,
	step: I16Vec3,
	length_max: Vec3,
	t_delta: Vec3,
	t: f32,
}

impl DDAState {
	fn new(start: Vec3, dir: Vec3, entry_t: f32) -> Self {
		let entry_point = start + dir * entry_t.max(0.0);
		let pos = entry_point.floor().as_i16vec3();
		let step = I16Vec3::new(
			if dir.x >= 0.0 { 1 } else { -1 },
			if dir.y >= 0.0 { 1 } else { -1 },
			if dir.z >= 0.0 { 1 } else { -1 },
		);
		let next_boundary = Vec3::new(
			if dir.x >= 0.0 { (pos.x + 1) as f32 } else { pos.x as f32 },
			if dir.y >= 0.0 { (pos.y + 1) as f32 } else { pos.y as f32 },
			if dir.z >= 0.0 { (pos.z + 1) as f32 } else { pos.z as f32 },
		);
		let t_delta = (Vec3::ONE / dir).abs();
		let length_max = (next_boundary - start) / dir;
		Self { pos, step, length_max, t_delta, t: entry_t.max(0.0) }
	}

	// Advance to next cell, stepping `scale` units on the chosen axis
	fn advance(&mut self, scale: i16) {
		let s = scale as f32;
		if self.length_max.x < self.length_max.y {
			if self.length_max.x < self.length_max.z {
				self.t = self.length_max.x;
				self.pos.x += self.step.x * scale;
				self.length_max.x += self.t_delta.x * s;
			} else {
				self.t = self.length_max.z;
				self.pos.z += self.step.z * scale;
				self.length_max.z += self.t_delta.z * s;
			}
		} else if self.length_max.y < self.length_max.z {
			self.t = self.length_max.y;
			self.pos.y += self.step.y * scale;
			self.length_max.y += self.t_delta.y * s;
		} else {
			self.t = self.length_max.z;
			self.pos.z += self.step.z * scale;
			self.length_max.z += self.t_delta.z * s;
		}
	}
}
