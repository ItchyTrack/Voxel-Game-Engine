use std::mem::{replace};
use std::fmt::Debug;

use glam::{I16Vec3, U16Vec3, Quat, Vec3, Vec4};

use crate::debug_draw;
use crate::pose::Pose;

const SIZE: u16 = 2;
const SIZE_USIZE: usize = SIZE as usize;
const SIZE_USIZE_CUBED: usize = SIZE_USIZE * SIZE_USIZE * SIZE_USIZE;

#[derive(Clone, Copy, Debug)]
pub enum ChildCell<T: Copy + Clone + Debug>
{
    None,
    Node { node: u32 },
    Data { data: T },
}

#[derive(Debug)]
struct GridTreeNode<T: Copy + Clone + Debug> {
    contents: [ChildCell<T>; SIZE_USIZE_CUBED],
	child_count: u16,
	parent: Option<u32>,
	pos: I16Vec3,
	scale: u16,
}

impl<T: Copy + Clone + Debug> GridTreeNode<T> {
	fn new(parent: Option<u32>, pos: I16Vec3, scale: u16) -> Self {
		Self {
			contents: [ChildCell::None; SIZE_USIZE_CUBED],
			child_count: 0,
			parent: parent,
			pos: pos,
			scale: scale,
		}
    }
	fn get_child_index(pos: &U16Vec3) -> u16 {
		pos.x + pos.y * SIZE + pos.z * SIZE * SIZE
	}
	fn get_child_cell(&self, pos: &U16Vec3) -> &ChildCell<T> {
        &self.contents[Self::get_child_index(pos) as usize]
    }
	fn get_child_cell_mut(&mut self, pos: &U16Vec3) -> &mut ChildCell<T> {
		&mut self.contents[Self::get_child_index(pos) as usize]
    }
	fn get_child_cell_pos(&self, index: u16) -> U16Vec3 {
		U16Vec3::new(index % SIZE, (index / SIZE) % SIZE, index / (SIZE * SIZE))
	}
	fn get_child_cell_world_pos(&self, index: u16) -> I16Vec3 {
		(self.get_child_cell_pos(index) * self.child_wold_size()).as_i16vec3() + self.pos
	}
	fn child_wold_size(&self) -> u16 {
		self.scale
	}
	fn world_size(&self) -> u16 {
		self.scale * SIZE
	}
}

#[derive(Debug)]
pub struct GridTree<T: Copy + Clone + Debug> {
	nodes: Vec<Option<GridTreeNode<T>>>,
	root: u32,
	item_count: u64,
}

impl<T: Copy + Clone + Debug> GridTree<T> {
    pub fn new() -> Self {
		Self {
			nodes: vec![Some(GridTreeNode::new(None, I16Vec3::splat(SIZE as i16 / -2 + 1), 1))],
			root: 0,
			item_count: 0,
		}
    }
	fn add_node(&mut self, parent: Option<u32>, pos: I16Vec3, scale: u16) -> u32 {
		assert!(self.get_next_added_node_index() == self.nodes.len() as u32);
		self.nodes.push(Some(GridTreeNode::new(parent, pos, scale)));
		self.nodes.len() as u32 - 1
	}
	fn remove_node(&mut self, index: u32) {
		let node = replace(&mut self.nodes[index as usize], None).unwrap();
		let mut iter = node.contents.iter().filter_map(|child| match child {
			ChildCell::Node { node } => Some(*node),
			_ => None
		});
		for _ in 0..node.child_count {
			self.remove_node(iter.next().unwrap());
		}
	}
	fn get_next_added_node_index(&self) -> u32 {
		self.nodes.len() as u32
	}
	fn get_node(&self, index: u32) -> &GridTreeNode<T> {
		self.nodes.get(index as usize).unwrap().as_ref().unwrap()
	}
	fn get_node_mut(&mut self, index: u32) -> &mut GridTreeNode<T> {
		self.nodes.get_mut(index as usize).unwrap().as_mut().unwrap()
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
			let local_pos = upos / node_size;
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
		let root_size = root.scale * SIZE;
		let mut upos = root_relative_pos.as_u16vec3();
		if upos.x >= root_size || upos.y >= root_size || upos.z >= root_size { return None; }
		let mut index = self.root;
		loop {
			let node = self.get_node(index);
			let node_size = node.scale;
			let local_pos = upos / node_size;
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
	pub fn insert(&mut self, pos: I16Vec3, t: T) -> Option<T> {
		let root = self.get_node(self.root);
		let mut root_world_pos = root.pos;
		let mut root_scale = root.scale;
		let mut root_relative_pos = pos - root_world_pos;
		let mut root_size = root_scale * SIZE;
		let mut upos = root_relative_pos.as_u16vec3();
		while root_relative_pos.is_negative_bitmask() != 0 || upos.x >= root_size || upos.y >= root_size || upos.z >= root_size {
			let old_root = self.root;
			let new_root_pos = SIZE / 2 - (root_relative_pos.is_negative_bitmask() == 0) as u16;
			root_world_pos = root_world_pos - I16Vec3::splat((root_size * new_root_pos) as i16);
			self.root = self.add_node(None, root_world_pos, root_scale * SIZE);
			self.get_node_mut(old_root).parent = Some(self.root);
			let new_root_node = self.get_node_mut(self.root);
			new_root_node.child_count += 1;
			*new_root_node.get_child_cell_mut(&U16Vec3::splat(new_root_pos)) = ChildCell::Node { node: old_root };
			root_relative_pos = pos - root_world_pos;
			upos = root_relative_pos.as_u16vec3();
			root_scale = root_size;
			root_size *= SIZE;
		}
		let mut index = self.root;
		loop {
			let node = self.get_node_mut(index);
			let node_scale = node.scale;
			let local_pos = upos / node_scale;
			if node.scale == 1 {
				node.child_count += 1;
				let cell = node.get_child_cell_mut(&local_pos);
				match cell {
					ChildCell::None => {
						*cell = ChildCell::Data { data: t };
						self.item_count += 1;
						return None;
					},
					ChildCell::Node { node: _ } => unreachable!(),
					ChildCell::Data { data } => {
						return Some(replace(data, t));
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
					index = self.add_node(Some(index), node_pos + (node_scale * local_pos).as_i16vec3(), node_scale / SIZE);
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
					index = self.add_node(Some(index), node_pos + (node_scale * local_pos).as_i16vec3(), node_scale / SIZE);
				}
			};
		}
	}
	pub fn len(&self) -> u64 { return self.item_count; }
	pub fn is_empty(&self) -> bool { return self.item_count == 0; }
	pub fn iter<'a>(&'a self) -> GridTreeIterator<'a, T> {
		GridTreeIterator::<'a>::new(self)
	}
	pub fn render_debug(&self, pose: &Pose) {
		let mut stack = vec![self.root];
		while let Some(idx) = stack.pop() {
			let node = self.nodes[idx as usize].as_ref().unwrap();
			let node_size = node.scale * SIZE;
			debug_draw::rectangular_prism(&(pose * Pose::new(node.pos.as_vec3(), Quat::IDENTITY)), Vec3::splat(node_size as f32), &Vec4::new(1.0, 1.0, 1.0, 0.0075), true);
			if node.child_count == 0 {
				continue;
			} else {
				for i in 0..SIZE.pow(3) as u32 {
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
}

pub struct GridTreeIterator<'a, T: Copy + Clone + Debug> {
	tree: &'a GridTree<T>,
	current_node: u32,
	current_index: u16,
}

impl<'a, T: Copy + Clone + Debug> GridTreeIterator<'a, T> {
	pub fn new(tree: &'a GridTree<T>) -> Self {
		Self {
			tree: tree,
			current_node: tree.root,
			current_index: 0,
		}
	}
}

impl<'a, T: Copy + Clone + Debug> Iterator for GridTreeIterator<'a, T> {
	type Item = (I16Vec3, &'a T);

	fn next(&mut self) -> Option<Self::Item> {
		let mut node: &GridTreeNode<T> = self.tree.get_node(self.current_node);
		loop {
			if node.child_count == 0 {
				if node.parent.is_none() { return None; }
				let pos = node.pos;
				self.current_node = node.parent.unwrap();
				node = self.tree.get_node(self.current_node);
				self.current_index = GridTreeNode::<T>::get_child_index(&((pos - node.pos).as_u16vec3() / node.scale)) + 1;
			} else {
				for i in self.current_index..SIZE.pow(3) as u16 {
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
					self.current_index = GridTreeNode::<T>::get_child_index(&((pos - node.pos).as_u16vec3() / node.scale)) + 1;
				}
			}
		}
	}
}

impl<'a, T: Copy + Clone + Debug> IntoIterator for &'a GridTree<T> {
    type Item = (I16Vec3, &'a T);
    type IntoIter = GridTreeIterator<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}
