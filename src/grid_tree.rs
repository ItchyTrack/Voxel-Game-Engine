use std::mem::{replace};
use std::fmt::Debug;

use glam::{IVec3, Quat, UVec3, Vec3, Vec4};

use crate::debug_draw;

#[derive(Clone, Debug)]
pub enum ChildCell<const SIZE: u32, T: Clone + Debug>
{
    None,
    Node { node: u32 },
    Data { data: T },
}

#[derive(Debug)]
struct GridTreeNode<const SIZE: u32, T: Clone + Debug> {
    contents: Vec<ChildCell<SIZE, T>>, // wish this was an array :(
	child_count: u32,
	parent: Option<u32>,
	pos: IVec3,
	depth: u32, // lower depth means deeper into the tree
}

impl<const SIZE: u32, T: Clone + Debug> GridTreeNode<SIZE, T> {
	fn new(parent: Option<u32>, pos: IVec3, depth: u32) -> Self {
		Self {
			contents: vec![ChildCell::None; SIZE.pow(3) as usize],
			child_count: 0,
			parent: parent,
			pos: pos,
			depth: depth,
		}
    }
	fn get_child_index(pos: &UVec3) -> u32 {
		pos.x + pos.y * SIZE + pos.z * SIZE.pow(2)
	}
	fn get_child_cell(&self, pos: &UVec3) -> &ChildCell<SIZE, T> {
        self.contents.get(Self::get_child_index(pos) as usize).unwrap()
    }
	fn get_child_cell_mut(&mut self, pos: &UVec3) -> &mut ChildCell<SIZE, T> {
		self.contents.get_mut(Self::get_child_index(pos) as usize).unwrap()
    }
	fn get_child_cell_pos(&self, index: u32) -> UVec3 {
		UVec3::new(index as u32 % SIZE, (index as u32 / SIZE) % SIZE, index as u32 / (SIZE * SIZE))
	}
	fn get_child_cell_scaled_pos(&self, index: u32) -> UVec3 {
		UVec3::new(index as u32 % SIZE, (index as u32 / SIZE) % SIZE, index as u32 / (SIZE * SIZE)) * SIZE.pow(self.depth)
	}
}

#[derive(Debug)]
pub struct GridTree<const SIZE: u32, T: Clone + Debug> {
	nodes: Vec<Option<GridTreeNode<SIZE, T>>>,
	root: u32,
	item_count: u64,
}

impl<const SIZE: u32, T: Clone + Debug> GridTree<SIZE, T> {
    pub fn new() -> Self {
		Self {
			nodes: vec![Some(GridTreeNode::new(None, IVec3::splat(SIZE as i32 / -2), 0))],
			root: 0,
			item_count: 0,
		}
    }
	fn add_node(&mut self, parent: Option<u32>, pos: IVec3, depth: u32) -> u32 {
		assert!(self.get_next_added_node_index() == self.nodes.len() as u32);
		self.nodes.push(Some(GridTreeNode::new(parent, pos, depth)));
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
	fn get_node(&self, index: u32) -> &GridTreeNode<SIZE, T> {
		self.nodes.get(index as usize).unwrap().as_ref().unwrap()
	}
	fn get_node_mut(&mut self, index: u32) -> &mut GridTreeNode<SIZE, T> {
		self.nodes.get_mut(index as usize).unwrap().as_mut().unwrap()
	}
	pub fn remove(&mut self, pos: &IVec3) -> Option<T> {
		let root = self.get_node(self.root);
		let root_relative_pos = pos - root.pos;
		if root_relative_pos.is_negative_bitmask() != 0 { return None; }
		let root_size = SIZE.pow(root.depth + 1);
		let mut upos = root_relative_pos.as_uvec3();
		if upos.x >= root_size || upos.y >= root_size || upos.z >= root_size { return None; }
		let mut index = self.root;
		loop {
			let node = self.get_node_mut(index);
			let node_size = SIZE.pow(node.depth);
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
	pub fn get_cell(&self, pos: &IVec3) -> Option<&ChildCell<SIZE, T>> {
		let root = self.get_node(self.root);
		let root_relative_pos = pos - root.pos;
		if root_relative_pos.is_negative_bitmask() != 0 { return None; }
		let root_size = SIZE.pow(root.depth + 1);
		let mut upos = root_relative_pos.as_uvec3();
		if upos.x >= root_size || upos.y >= root_size || upos.z >= root_size { return None; }
		let mut index = self.root;
		loop {
			let node = self.get_node(index);
			let node_size = SIZE.pow(node.depth);
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
	pub fn get(&self, pos: &IVec3) -> Option<&T> {
		match self.get_cell(&pos) {
			Some( value ) => match value {
				crate::grid_tree::ChildCell::Data { data } => return Some(data),
				_ => None,
			},
			None => None,
		}
	}
	pub fn contains_key(&self, pos: &IVec3) -> bool {
		self.get(pos).is_some()
	}
	pub fn insert(&mut self, pos: IVec3, t: T) -> Option<T> {
		let root = self.get_node(self.root);
		let mut root_pos = root.pos;
		let mut root_depth = root.depth;
		let mut root_relative_pos = pos - root_pos;
		// println!("root_relative_pos: {}. {}. {}", root_relative_pos, pos, root_pos);
		let mut root_size = SIZE.pow(root.depth + 1) as i32;
		// println!("root_size {}, {}", root_size, root_relative_pos);
		while root_relative_pos.is_negative_bitmask() != 0 || root_relative_pos.x >= root_size || root_relative_pos.y >= root_size || root_relative_pos.z >= root_size {
			let old_root = self.root;
			self.root = self.add_node(None, IVec3::splat(root_size * (SIZE as i32 / -2 + (root_depth as i32 % 2))) + root_pos, root_depth + 1);
			self.get_node_mut(old_root).parent = Some(self.root);
			let new_root_node = self.get_node_mut(self.root);
			new_root_node.child_count += 1;
			*new_root_node.get_child_cell_mut(&UVec3::splat(SIZE / 2 - (root_depth % 2))) = ChildCell::Node { node: old_root };
			root_pos = IVec3::splat(root_size * (SIZE as i32 / -2 + (root_depth as i32 % 2))) + root_pos;
			root_depth += 1;
			root_relative_pos = pos - root_pos;
			root_size *= SIZE as i32;
		}
		let mut upos = root_relative_pos.as_uvec3();
		let mut index = self.root;
		loop {
			let node = self.get_node_mut(index);
			let node_scale = SIZE.pow(node.depth);
			let local_pos = upos / node_scale;
			if node.depth == 0 {
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
			let node_depth = node.depth;
			let node = self.get_node_mut(index);
			let cell = node.get_child_cell_mut(&local_pos);
			match cell {
				ChildCell::None => {
					unsafe {
						let ptr = cell as *mut ChildCell<SIZE, T>;
						node.child_count += 1;
						*ptr = ChildCell::Node { node: self.get_next_added_node_index() };
					};
					index = self.add_node(Some(index), node_pos + (node_scale * local_pos).as_ivec3(), node_depth - 1);
				},
				ChildCell::Node { node: child_node } => {
					index = *child_node;
				}
				ChildCell::Data { data: _ } => { // TODO: fill rest of node with data.
					unsafe {
						let ptr = cell as *mut ChildCell<SIZE, T>;
						node.child_count += 1;
						*ptr = ChildCell::Node { node: self.get_next_added_node_index() };
					};
					index = self.add_node(Some(index), node_pos + (node_scale * local_pos).as_ivec3(), node_depth - 1);
				}
			};
		}
	}
	pub fn len(&self) -> u64 { return self.item_count; }
	pub fn is_empty(&self) -> bool { return self.item_count == 0; }
	pub fn iter<'a>(&'a self) -> GridTreeIterator<'a, SIZE, T> {
		GridTreeIterator::<'a>::new(self)
	}
	pub fn render_debug(&self, pos: Vec3, rot: &Quat) {
		let mut stack = vec![self.root];
		while let Some(idx) = stack.pop() {
			let node = self.nodes[idx as usize].as_ref().unwrap();
			let node_size = SIZE.pow(node.depth+1) as i32;
			debug_draw::quad(
				rot * node.pos.as_vec3() + pos,
				rot * (node.pos + IVec3::X * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::X * node_size as i32 + IVec3::Y * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::Y * node_size as i32).as_vec3() + pos,
				Vec4::ONE,
				false
			);
			debug_draw::quad(
				rot * node.pos.as_vec3() + pos,
				rot * (node.pos + IVec3::X * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::X * node_size as i32 + IVec3::Z * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::Z * node_size as i32).as_vec3() + pos,
				Vec4::ONE,
				false
			);
			debug_draw::quad(
				rot * node.pos.as_vec3() + pos,
				rot * (node.pos + IVec3::Y * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::Y * node_size as i32 + IVec3::Z * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::Z * node_size as i32).as_vec3() + pos,
				Vec4::ONE,
				false
			);
			debug_draw::quad(
				rot * (node.pos + IVec3::ONE * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::Z * node_size as i32 + IVec3::Y * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::Z * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::Z * node_size as i32 + IVec3::X * node_size as i32).as_vec3() + pos,
				Vec4::ONE,
				false
			);
			debug_draw::quad(
				rot * (node.pos + IVec3::ONE * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::Z * node_size as i32 + IVec3::Y * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::Y * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::X * node_size as i32 + IVec3::Y * node_size as i32).as_vec3() + pos,
				Vec4::ONE,
				false
			);
			debug_draw::quad(
				rot * (node.pos + IVec3::ONE * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::Z * node_size as i32 + IVec3::X * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::X * node_size as i32).as_vec3() + pos,
				rot * (node.pos + IVec3::X * node_size as i32 + IVec3::Y * node_size as i32).as_vec3() + pos,
				Vec4::ONE,
				false
			);
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

pub struct GridTreeIterator<'a, const SIZE: u32, T: Clone + Debug> {
	tree: &'a GridTree<SIZE, T>,
	current_node: u32,
	current_index: u32,
}

impl<'a, const SIZE: u32, T: Clone + Debug> GridTreeIterator<'a, SIZE, T> {
	pub fn new(tree: &'a GridTree<SIZE, T>) -> Self {
		Self {
			tree: tree,
			current_node: tree.root,
			current_index: 0,
		}
	}
}

impl<'a, const SIZE: u32, T: Clone + Debug> Iterator for GridTreeIterator<'a, SIZE, T> {
	type Item = (IVec3, &'a T);

	fn next(&mut self) -> Option<Self::Item> {
		let mut node: &GridTreeNode<SIZE, T> = self.tree.get_node(self.current_node);
		loop {
			if node.child_count == 0 {
				if node.parent.is_none() { return None; }
				let pos = node.pos;
				self.current_node = node.parent.unwrap();
				node = self.tree.get_node(self.current_node);
				self.current_index = GridTreeNode::<SIZE, T>::get_child_index(&((pos - node.pos).as_uvec3() / SIZE.pow(node.depth))) + 1;
			} else {
				for i in self.current_index..SIZE.pow(3) as u32 {
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
								node.get_child_cell_scaled_pos(i).as_ivec3() + node.pos,
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
					self.current_index = GridTreeNode::<SIZE, T>::get_child_index(&((pos - node.pos).as_uvec3() / SIZE.pow(node.depth))) + 1;
				}
			}
		}
	}
}

impl<'a, const SIZE: u32, T: Clone + Debug> IntoIterator for &'a GridTree<SIZE, T> {
    type Item = (IVec3, &'a T);
    type IntoIter = GridTreeIterator<'a, SIZE, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}
