use std::fmt::Debug;
use std::marker::PhantomData;

use bevy::math::{I8Vec3, IVec3, U8Vec3, UVec3, Vec3};
use bevy::transform::components::Transform;

pub const LOG_SIZE: u8 = 2;
pub const SIZE: u8 = 1u8 << LOG_SIZE;
pub const SIZE_CUBED: u8 = SIZE * SIZE * SIZE;
pub const SIZE_USIZE: usize = SIZE as usize;
pub const SIZE_USIZE_CUBED: usize = SIZE_USIZE * SIZE_USIZE * SIZE_USIZE;

pub const MAX_TREE_DEPTH: u8 = 10; // it is lower than this but im being safe
pub const MAX_TREE_DEPTH_USIZE: usize = MAX_TREE_DEPTH as usize;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CellKind {
	Empty,
	Data,
	Node,
}

/// How a single cell of a node is encoded (empty / data leaf / child-node offset).
pub trait GridCell: Copy + Eq + Debug {
	type Data: Copy + Eq + Ord + Debug;
	const EMPTY: Self;
	const MAX_DATA: Self::Data;
	const MAX_NODE_OFFSET: u32;
	fn data(value: Self::Data) -> Self;
	fn node(offset: u32) -> Self;
	fn kind(self) -> CellKind;
	fn data_value(self) -> Self::Data;
	fn node_offset(self) -> u32;
}

/// The coordinate system a tree is keyed in. The tree stores and traverses
/// everything in `IVec3`/`u32` internally; this trait only bridges the public
/// position/size types at the API boundary, plus the per-system depth cap.
pub trait GridCoord: Copy + Debug + 'static {
	/// Public signed world-space position vector (e.g. `I16Vec3` or `IVec3`).
	type Pos: Copy + Eq + Debug;
	/// Public size scalar yielded for cells (e.g. `u16` or `u32`).
	type Size: Copy + Eq + Ord + Debug;
	/// Highest root depth allowed before an out-of-range insert is skipped.
	const MAX_ROOT_DEPTH: u8;
	fn to_ivec3(pos: Self::Pos) -> IVec3;
	fn from_ivec3(v: IVec3) -> Self::Pos;
	fn size_from_u32(s: u32) -> Self::Size;
}

/// `i16` coordinate system (world voxels).
#[derive(Clone, Copy, Debug)]
pub struct I16Coord;
impl GridCoord for I16Coord {
	type Pos = bevy::math::I16Vec3;
	type Size = u16;
	// size(7) = 1<<16 is the first depth no i16 span can reach, so 6 is the cap.
	const MAX_ROOT_DEPTH: u8 = 6;
	fn to_ivec3(pos: Self::Pos) -> IVec3 {
		pos.as_ivec3()
	}
	fn from_ivec3(v: IVec3) -> Self::Pos {
		v.as_i16vec3()
	}
	fn size_from_u32(s: u32) -> Self::Size {
		s as u16
	}
}

/// `i32` coordinate system (chunk-space and other coarse grids).
#[derive(Clone, Copy, Debug)]
pub struct I32Coord;
impl GridCoord for I32Coord {
	type Pos = IVec3;
	type Size = u32;
	// size(15) = 1<<32 overflows u32; the node-arena cap bounds us long before this.
	const MAX_ROOT_DEPTH: u8 = 13;
	fn to_ivec3(pos: Self::Pos) -> IVec3 {
		pos
	}
	fn from_ivec3(v: IVec3) -> Self::Pos {
		v
	}
	fn size_from_u32(s: u32) -> Self::Size {
		s
	}
}

pub fn size(node_depth: u8) -> u32 {
	1 << (LOG_SIZE * (node_depth + 1))
}
pub fn child_size(node_depth: u8) -> u32 {
	1 << (LOG_SIZE * node_depth)
}

#[derive(Debug, Clone)]
pub struct GridTreeNode<C: GridCell> {
	pub contents: [C; SIZE_USIZE_CUBED],
	pub parent_offset: u16, // if parent_offset == 0 then no parent
	pub used_cell_count: u8,
}

pub fn get_child_contents_index(contents_pos: U8Vec3) -> u8 {
	contents_pos.x + contents_pos.y * SIZE + contents_pos.z * SIZE * SIZE
}

pub fn get_child_contents_pos(contents_index: u8) -> U8Vec3 {
	U8Vec3::new(contents_index % SIZE, (contents_index / SIZE) % SIZE, contents_index / (SIZE * SIZE))
}

impl<C: GridCell> GridTreeNode<C> {
	fn new_root() -> Self {
		Self::new(0)
	}
	fn new(parent_offset: u16) -> Self {
		Self { contents: [C::EMPTY; SIZE_USIZE_CUBED], parent_offset, used_cell_count: 0 }
	}
	fn get_parent_offset(&self) -> Option<u16> {
		if self.parent_offset == 0 { None } else { Some(self.parent_offset) }
	}
	fn get_child_cell_from_index(&self, contents_index: u8) -> C {
		self.contents[contents_index as usize]
	}
	fn get_child_cell(&self, contents_pos: U8Vec3) -> C {
		self.get_child_cell_from_index(get_child_contents_index(contents_pos))
	}
	fn set_child_cell_from_index(&mut self, contents_index: u8, cell: C) {
		self.contents[contents_index as usize] = cell;
	}
	fn set_child_cell(&mut self, contents_pos: U8Vec3, cell: C) {
		self.set_child_cell_from_index(get_child_contents_index(contents_pos), cell)
	}
	fn set_child_cell_to_none_from_index(&mut self, contents_index: u8) {
		self.set_child_cell_from_index(contents_index, C::EMPTY);
	}
	fn set_child_cell_to_none(&mut self, contents_pos: U8Vec3) {
		self.set_child_cell_to_none_from_index(get_child_contents_index(contents_pos))
	}
	fn set_child_cell_to_data(&mut self, contents_pos: U8Vec3, data: C::Data) {
		self.set_child_cell_from_index(get_child_contents_index(contents_pos), C::data(data));
	}
	fn set_child_cell_to_node_from_index(&mut self, contents_index: u8, node_offset: u32) {
		self.set_child_cell_from_index(contents_index, C::node(node_offset));
	}
	fn set_child_cell_to_node(&mut self, contents_pos: U8Vec3, node_offset: u32) {
		self.set_child_cell_to_node_from_index(get_child_contents_index(contents_pos), node_offset)
	}
}

#[derive(Debug, Clone)]
pub struct GridTree<C: GridCell, Co: GridCoord> {
	nodes: Vec<GridTreeNode<C>>, // root at 0
	root_pos: IVec3,
	root_depth: u8,
	item_count: u64,
	dead_nodes: usize,
	_coord: PhantomData<Co>,
}

impl<C: GridCell, Co: GridCoord> GridTree<C, Co> {
	pub fn new() -> Self {
		Self { nodes: vec![GridTreeNode::new_root()], root_pos: IVec3::ZERO, root_depth: 0, item_count: 0, dead_nodes: 0, _coord: PhantomData }
	}

	pub fn get(&self, pos: &Co::Pos) -> Option<C::Data> {
		self.get_internal(Co::to_ivec3(*pos))
	}

	fn get_internal(&self, pos: IVec3) -> Option<C::Data> {
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
		let mut current_node_index: u32 = 0;
		let mut current_relative_pos = root_relative_pos;
		let mut current_depth = self.root_depth;
		loop {
			let node = &self.nodes[current_node_index as usize];
			let contents_pos = (current_relative_pos / child_size(current_depth)).as_u8vec3();
			let cell = node.get_child_cell(contents_pos);
			match cell.kind() {
				CellKind::Empty => return None,
				CellKind::Data => return Some(cell.data_value()),
				CellKind::Node => {
					current_relative_pos %= child_size(current_depth);
					current_node_index += cell.node_offset();
					current_depth -= 1;
				}
			}
		}
	}

	pub fn contains_key(&self, pos: &Co::Pos) -> bool {
		self.get(pos).is_some()
	}

	pub fn is_area_filled(&self, pos: &Co::Pos, size: IVec3) -> bool {
		let base = Co::to_ivec3(*pos);
		for x in 0..size.x {
			for y in 0..size.y {
				for z in 0..size.z {
					if self.get_internal(base + IVec3::new(x, y, z)).is_none() {
						return false;
					}
				}
			}
		}
		true
	}

	pub fn add_area(&mut self, pos: &Co::Pos, size: IVec3, data: C::Data) {
		let base = Co::to_ivec3(*pos);
		for x in 0..size.x {
			for y in 0..size.y {
				for z in 0..size.z {
					self.insert(&Co::from_ivec3(base + IVec3::new(x, y, z)), data);
				}
			}
		}
	}

	pub fn remove_area(&mut self, pos: &Co::Pos, size: IVec3) {
		let base = Co::to_ivec3(*pos);
		for x in 0..size.x {
			for y in 0..size.y {
				for z in 0..size.z {
					self.remove(&Co::from_ivec3(base + IVec3::new(x, y, z)));
				}
			}
		}
	}

	fn make_sure_root_covers_pos(&mut self, pos: IVec3) -> bool {
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

	fn root_covers(&self, pos: IVec3) -> bool {
		let r = pos - self.root_pos;
		r.min_element() >= 0 && (r.max_element() as i64) < size(self.root_depth) as i64
	}

	fn rebuild_to_cover(&mut self, pos: IVec3) -> bool {
		let mut min = pos;
		let mut max = pos;
		let mut voxels: Vec<(IVec3, C::Data)> = Vec::new();
		self.each_leaf(|origin, cell_size, value| {
			min = min.min(origin);
			max = max.max(origin + IVec3::splat(cell_size as i32 - 1));
			for dx in 0..cell_size as i32 {
				for dy in 0..cell_size as i32 {
					for dz in 0..cell_size as i32 {
						voxels.push((origin + IVec3::new(dx, dy, dz), value));
					}
				}
			}
		});
		let span = (max - min).max_element() as i64 + 1;
		let mut depth = 0u8;
		while (size(depth) as i64) < span {
			if depth >= Co::MAX_ROOT_DEPTH {
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
			self.insert_into_covered(vp, value);
		}
		true
	}

	/// Visit every DATA leaf as (world origin, cell size, value) via an internal DFS.
	fn each_leaf(&self, mut f: impl FnMut(IVec3, u32, C::Data)) {
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
	fn set_voxel_in_data_cell(&mut self, parent_node_index: u32, parent_depth: u8, current_cell: C, cell_to_set: C, pos: UVec3) {
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
	fn set_voxel_in_none_cell(&mut self, parent_node_index: u32, parent_depth: u8, cell_to_set: C, pos: UVec3) {
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
	fn try_merge(&mut self, node_index: u32, data: C::Data, cell_index_stack: &[u8]) {
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
	fn try_merge_empty(&mut self, node_index: u32, cell_index_stack: &[u8]) {
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
	fn has_node_budget(&mut self) -> bool {
		self.maybe_compact();
		if self.nodes.len() + MAX_TREE_DEPTH_USIZE <= C::MAX_NODE_OFFSET as usize {
			return true;
		}
		bevy::log::warn!("GridTree node arena full ({} live); skipping edit", self.nodes.len());
		false
	}

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
	fn insert_into_covered(&mut self, pos: IVec3, data: C::Data) -> Option<C::Data> {
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
					current_node_index += cell.node_offset();
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
					current_node_index += cell.node_offset();
					current_depth -= 1;
				}
			}
			cell_index_stack_size += 1;
		}
	}

	pub fn len(&self) -> u64 {
		return self.item_count;
	}
	pub fn is_empty(&self) -> bool {
		return self.item_count == 0;
	}
	pub fn iter(&self) -> GridTreeIterator<'_, C, Co> {
		GridTreeIterator::new(self)
	}

	/// Visit every DATA leaf whose cell box intersects the inclusive region
	/// `[min, max]`, pruning subtrees that don't overlap. `f` receives
	/// (cell origin, cell size, value). Far cheaper than `iter` when the region
	/// is small relative to the tree.
	pub fn for_each_in_region(&self, min: Co::Pos, max: Co::Pos, mut f: impl FnMut(Co::Pos, Co::Size, C::Data)) {
		if self.is_empty() {
			return;
		}
		let (min, max) = (Co::to_ivec3(min), Co::to_ivec3(max));
		self.region_recurse(0, self.root_depth, self.root_pos, min, max, &mut |o, s, v| f(Co::from_ivec3(o), Co::size_from_u32(s), v));
	}

	fn region_recurse(&self, node_index: u32, node_depth: u8, node_origin: IVec3, min: IVec3, max: IVec3, f: &mut dyn FnMut(IVec3, u32, C::Data)) {
		let node = &self.nodes[node_index as usize];
		let cell_size = child_size(node_depth);
		for i in 0..SIZE_CUBED {
			let cell = node.contents[i as usize];
			if cell.kind() == CellKind::Empty {
				continue;
			}
			let contents_pos = get_child_contents_pos(i);
			let child_origin = node_origin + (contents_pos.as_uvec3() * cell_size).as_ivec3();
			let child_end = child_origin + IVec3::splat(cell_size as i32); // exclusive
			if child_origin.cmpgt(max).any() || child_end.cmple(min).any() {
				continue;
			}
			match cell.kind() {
				CellKind::Data => f(child_origin, cell_size, cell.data_value()),
				CellKind::Node => self.region_recurse(node_index + cell.node_offset(), node_depth - 1, child_origin, min, max, f),
				CellKind::Empty => unreachable!(),
			}
		}
	}

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

		if tmax < 0.0 || tmin > tmax {
			return None;
		}

		Some(tmin)
	}

	pub fn raycast(&self, transform: &Transform, max_length: Option<f32>) -> Option<(Co::Pos, I8Vec3, f32)> {
		let max_length = max_length.unwrap_or(f32::MAX);

		let origin = transform.translation;
		let dir = transform.rotation * Vec3::Z;

		let root_min = self.root_pos.as_vec3();
		let root_max = root_min + Vec3::splat(size(self.root_depth) as f32);

		// Ray vs root AABB
		let distance_to_aabb = match Self::ray_aabb_intersection(&origin, &dir, &(root_min, root_max)) {
			Some(dis) => dis,
			None => return None,
		};
		let post_aabb_origin_pre_shift = origin + dir * distance_to_aabb;
		let post_aabb_origin = post_aabb_origin_pre_shift
			.min(self.root_pos.as_vec3() + Vec3::splat(((size(self.root_depth)) as f32) - 0.00001))
			.max(self.root_pos.as_vec3());
		let post_aabb_origin = post_aabb_origin.move_towards(post_aabb_origin.floor() + 0.5, 0.001);
		let root_relative_post_aabb_origin = post_aabb_origin - self.root_pos.as_vec3();
		let delta = dir.recip().abs();
		let step = dir.signum().as_i8vec3();
		let mut axis_distances = dir.recip()
			* Vec3::new(
				if step.x > 0 { root_relative_post_aabb_origin.x.ceil() } else { root_relative_post_aabb_origin.x.floor() }
					- root_relative_post_aabb_origin.x,
				if step.y > 0 { root_relative_post_aabb_origin.y.ceil() } else { root_relative_post_aabb_origin.y.floor() }
					- root_relative_post_aabb_origin.y,
				if step.z > 0 { root_relative_post_aabb_origin.z.ceil() } else { root_relative_post_aabb_origin.z.floor() }
					- root_relative_post_aabb_origin.z,
			)
			+ distance_to_aabb;
		let mut root_relative_grid_pos = root_relative_post_aabb_origin.as_uvec3();
		let mut last_step_axis = (post_aabb_origin_pre_shift - post_aabb_origin).abs().max_position() as u8;
		let mut current_node_index = 0u32;
		let mut current_depth = self.root_depth;
		let mut last_distance = distance_to_aabb;
		if max_length < last_distance {
			return None;
		}
		loop {
			let mut current_node = &self.nodes[current_node_index as usize];
			let node_relative_grid_pos = root_relative_grid_pos % size(current_depth);
			let contents_pos = (node_relative_grid_pos / child_size(current_depth)).as_u8vec3();
			let cell = current_node.get_child_cell(contents_pos);
			match cell.kind() {
				CellKind::Empty => {
					if child_size(current_depth) != 1 {
						let node_cell_relative_grid_pos = node_relative_grid_pos % child_size(current_depth);
						let mut step_amount = UVec3::select(
							step.cmpgt(I8Vec3::ZERO),
							UVec3::splat(child_size(current_depth) - 1) - node_cell_relative_grid_pos,
							node_cell_relative_grid_pos,
						);
						// step to edge of cell. step_amount is 0 if child_size is 0
						let distance_to_edge_of_cell = axis_distances + step_amount.as_vec3() * delta;
						match distance_to_edge_of_cell.min_position() {
							0 => {
								step_amount.y = ((distance_to_edge_of_cell.x - axis_distances.y + delta.y) / delta.y).abs() as u32;
								step_amount.z = ((distance_to_edge_of_cell.x - axis_distances.z + delta.z) / delta.z).abs() as u32;
							}
							1 => {
								step_amount.x = ((distance_to_edge_of_cell.y - axis_distances.x + delta.x) / delta.x).abs() as u32;
								step_amount.z = ((distance_to_edge_of_cell.y - axis_distances.z + delta.z) / delta.z).abs() as u32;
							}
							2 => {
								step_amount.x = ((distance_to_edge_of_cell.z - axis_distances.x + delta.x) / delta.x).abs() as u32;
								step_amount.y = ((distance_to_edge_of_cell.z - axis_distances.y + delta.y) / delta.y).abs() as u32;
							}
							_ => unreachable!(),
						}
						axis_distances += delta * step_amount.as_vec3();
						root_relative_grid_pos = (root_relative_grid_pos.as_ivec3() + step_amount.as_ivec3() * step.as_ivec3()).as_uvec3();
					}
					match axis_distances.min_position() {
						0 => {
							if max_length < axis_distances.x {
								return None;
							}
							let root_relative_grid_pos_x = root_relative_grid_pos.x as i64 + step.x as i64;
							if root_relative_grid_pos_x < 0 || root_relative_grid_pos_x >= size(self.root_depth) as i64 {
								return None;
							}
							let root_relative_grid_pos_x = root_relative_grid_pos_x as u32;
							loop {
								if root_relative_grid_pos.x / size(current_depth) == root_relative_grid_pos_x / size(current_depth) {
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
						}
						1 => {
							if max_length < axis_distances.y {
								return None;
							}
							let root_relative_grid_pos_y = root_relative_grid_pos.y as i64 + step.y as i64;
							if root_relative_grid_pos_y < 0 || root_relative_grid_pos_y >= size(self.root_depth) as i64 {
								return None;
							}
							let root_relative_grid_pos_y = root_relative_grid_pos_y as u32;
							loop {
								if root_relative_grid_pos.y / size(current_depth) == root_relative_grid_pos_y / size(current_depth) {
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
						}
						2 => {
							if max_length < axis_distances.z {
								return None;
							}
							let root_relative_grid_pos_z = root_relative_grid_pos.z as i64 + step.z as i64;
							if root_relative_grid_pos_z < 0 || root_relative_grid_pos_z >= size(self.root_depth) as i64 {
								return None;
							}
							let root_relative_grid_pos_z = root_relative_grid_pos_z as u32;
							loop {
								if root_relative_grid_pos.z / size(current_depth) == root_relative_grid_pos_z / size(current_depth) {
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
						}
						_ => unreachable!(),
					}
				}
				CellKind::Data => {
					return Some((
						Co::from_ivec3(root_relative_grid_pos.as_ivec3() + self.root_pos),
						-step.to_array()[last_step_axis as usize] * I8Vec3::AXES[last_step_axis as usize],
						last_distance,
					));
				}
				CellKind::Node => {
					current_depth -= 1;
					current_node_index += cell.node_offset();
				}
			}
		}
	}
}

impl<'a, C: GridCell, Co: GridCoord> IntoIterator for &'a GridTree<C, Co> {
	type Item = (Co::Pos, Co::Size, C::Data);
	type IntoIter = GridTreeIterator<'a, C, Co>;

	fn into_iter(self) -> Self::IntoIter {
		self.iter()
	}
}

pub struct GridTreeIterator<'a, C: GridCell, Co: GridCoord> {
	tree: &'a GridTree<C, Co>,
	stack: Vec<(u32, u8, u8, IVec3)>,
}

impl<'a, C: GridCell, Co: GridCoord> GridTreeIterator<'a, C, Co> {
	pub fn new(tree: &'a GridTree<C, Co>) -> Self {
		if tree.nodes.is_empty() {
			return Self { tree, stack: vec![] };
		}
		Self { tree, stack: vec![(0, 0, tree.root_depth, tree.root_pos)] }
	}
}

impl<'a, C: GridCell, Co: GridCoord> Iterator for GridTreeIterator<'a, C, Co> {
	type Item = (Co::Pos, Co::Size, C::Data);

	fn next(&mut self) -> Option<Self::Item> {
		loop {
			let (node_index, start_child, node_depth, node_origin) = self.stack.last_mut()?;
			let node_index = *node_index;
			let node_origin = *node_origin;
			let node_depth = *node_depth;

			let node = &self.tree.nodes[node_index as usize];
			let child_size = child_size(node_depth); // size of one cell in this node

			// Scan forward from where we left off
			let scan_start = *start_child;
			let mut found = false;

			for i in scan_start..SIZE_CUBED {
				let cell = node.contents[i as usize];
				let contents_pos = get_child_contents_pos(i);
				let child_world_origin = node_origin + (contents_pos.as_uvec3() * child_size).as_ivec3();

				match cell.kind() {
					CellKind::Empty => { /* skip */ }
					CellKind::Data => {
						// DATA leaf – yield it and resume after this cell next time
						*self.stack.last_mut().unwrap() = (node_index, i + 1, node_depth, node_origin);
						return Some((Co::from_ivec3(child_world_origin), Co::size_from_u32(child_size), cell.data_value()));
					}
					CellKind::Node => {
						// NODE – push child onto stack, restart inner loop from there
						let child_node_index = node_index + cell.node_offset();
						// Record that we should resume from i+1 when we pop back
						*self.stack.last_mut().unwrap() = (node_index, i + 1, node_depth, node_origin);
						self.stack.push((child_node_index, 0, node_depth - 1, child_world_origin));
						found = true;
						break;
					}
				}
			}

			if !found {
				// Exhausted this node – pop it
				self.stack.pop();
			}
		}
	}
}
