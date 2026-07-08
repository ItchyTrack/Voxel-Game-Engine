use std::marker::PhantomData;

use bevy::math::{I8Vec3, IVec3, U8Vec3, UVec3};
use bevy::transform::components::Transform;

mod cell;
mod coord;
mod raycast;
mod traversal;
mod view;
pub use cell::{CellKind, GridCell};
pub use coord::{GridCoord, U16Coord, U32Coord};
pub use view::{CellRef, ChildCells, GridTreeView, LeafCells, NodeRef};

pub const LOG_SIZE: u8 = 2;
pub const SIZE: u8 = 1u8 << LOG_SIZE;
pub const SIZE_CUBED: u8 = SIZE * SIZE * SIZE;
pub const SIZE_USIZE: usize = SIZE as usize;
pub const SIZE_USIZE_CUBED: usize = SIZE_USIZE * SIZE_USIZE * SIZE_USIZE;

pub const MAX_TREE_DEPTH: u8 = 10; // it is lower than this but im being safe
pub const MAX_TREE_DEPTH_USIZE: usize = MAX_TREE_DEPTH as usize;

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
		if self.parent_offset == 0 {
			None
		} else {
			Some(self.parent_offset)
		}
	}
	fn get_child_cell_from_index(&self, contents_index: u8) -> C {
		self.contents[contents_index as usize]
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
	fn set_child_cell_to_node_from_index(&mut self, contents_index: u8, node_index: u32) {
		self.set_child_cell_from_index(contents_index, C::node(node_index));
	}
	fn set_child_cell_to_node(&mut self, contents_pos: U8Vec3, node_index: u32) {
		self.set_child_cell_to_node_from_index(get_child_contents_index(contents_pos), node_index)
	}
}

#[derive(Debug, Clone)]
pub struct GridTree<C: GridCell, Co: GridCoord> {
	nodes: Vec<GridTreeNode<C>>, // root at 0
	root_pos: IVec3,
	root_depth: u8,
	item_count: u64,
	dead_nodes: usize,
	free_nodes: Vec<u32>,
	_coord: PhantomData<Co>,
}

#[derive(Clone, Copy, Debug)]
struct AreaOp<D: Copy> {
	min: IVec3,
	end: IVec3,
	data: D,
}

impl<C: GridCell, Co: GridCoord> GridTree<C, Co> {
	pub fn new() -> Self {
		Self { nodes: vec![GridTreeNode::new_root()], root_pos: IVec3::ZERO, root_depth: 0, item_count: 0, dead_nodes: 0, free_nodes: Vec::new(), _coord: PhantomData }
	}

	pub fn view(&self) -> GridTreeView<'_, C, Co> {
		GridTreeView::new(&self.nodes, self.root_pos, self.root_depth, self.item_count)
	}

	pub fn get(&self, pos: &Co::Pos) -> Option<C::Data> {
		traversal::get(self.view(), *pos)
	}

	pub fn contains_key(&self, pos: &Co::Pos) -> bool {
		self.get(pos).is_some()
	}

	pub fn is_region_filled(&self, region: GridRegion) -> bool {
		traversal::is_area_filled(self.view(), Co::from_ivec3(region.min), region.size())
	}

	pub fn is_area_filled(&self, pos: &Co::Pos, size: IVec3) -> bool {
		let Some(region) = GridRegion::from_min_size(Co::to_ivec3(*pos), size) else { return true };
		self.is_region_filled(region)
	}

	pub fn ensure_area_covered(&mut self, pos: &Co::Pos, size: IVec3) -> bool {
		if size.cmple(IVec3::ZERO).any() {
			return true;
		}
		let min = Co::to_ivec3(*pos);
		let max = min + size - IVec3::ONE;
		self.make_sure_root_covers_area(min, max)
	}
}

mod serialize;
mod bulk;
mod point_ops;
mod region;
mod region_ops;
mod single_build;
mod storage;
mod surgery;

pub use region::GridRegion;

impl<C: GridCell, Co: GridCoord> GridTree<C, Co> {
	pub fn len(&self) -> u64 {
		self.item_count
	}
	pub fn is_empty(&self) -> bool {
		self.item_count == 0
	}
	pub fn iter(&self) -> GridTreeIterator<'_, C, Co> {
		GridTreeIterator::new(self)
	}

	/// Visit every DATA leaf whose cell box intersects a half-open region.
	pub fn for_each_in_region(&self, region: GridRegion, f: impl FnMut(Co::Pos, Co::Size, C::Data)) {
		traversal::for_each_in_region(self.view(), Co::from_ivec3(region.min), Co::from_ivec3(region.max_inclusive()), f);
	}

	pub fn any_in_region(&self, region: GridRegion) -> bool {
		traversal::any_in_region(self.view(), Co::from_ivec3(region.min), Co::from_ivec3(region.max_inclusive()))
	}

	/// Visit every occupied cell (internal node or data leaf) whose cell box intersects a half-open region.
	pub fn for_each_node_in_region(&self, region: GridRegion, f: impl FnMut(Co::Pos, Co::Size, bool)) {
		traversal::for_each_node_in_region(self.view(), Co::from_ivec3(region.min), Co::from_ivec3(region.max_inclusive()), f);
	}

	pub fn for_each_occupied_tile_cover(&self, region: GridRegion, tile_size: i32, f: impl FnMut(IVec3)) {
		traversal::for_each_occupied_tile_cover(self.view(), Co::from_ivec3(region.min), Co::from_ivec3(region.max_inclusive()), tile_size, f)
	}

	pub fn raycast(&self, transform: &Transform, max_length: Option<f32>) -> Option<(Co::Pos, I8Vec3, f32)> {
		raycast::raycast(self.view(), transform, max_length)
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
	leaves: LeafCells<'a, C, Co>,
}

impl<'a, C: GridCell, Co: GridCoord> GridTreeIterator<'a, C, Co> {
	pub fn new(tree: &'a GridTree<C, Co>) -> Self {
		Self { leaves: tree.view().leaves() }
	}
}

impl<'a, C: GridCell, Co: GridCoord> Iterator for GridTreeIterator<'a, C, Co> {
	type Item = (Co::Pos, Co::Size, C::Data);

	fn next(&mut self) -> Option<Self::Item> {
		self.leaves.next().map(|leaf| (Co::from_ivec3(leaf.origin), Co::size_from_u32(leaf.size), leaf.data_value()))
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::voxel_grid_tree::PackedCell;
	use bevy::math::{U16Vec3, Vec3};

	#[test]
	fn add_area_preserves_large_runs() {
		let mut tree = GridTree::<PackedCell, U16Coord>::new();
		tree.add_area(&U16Vec3::ZERO, IVec3::splat(16), 7);
		assert_eq!(tree.len(), 16 * 16 * 16);
		assert_eq!(tree.get(&U16Vec3::new(0, 0, 0)), Some(7));
		assert_eq!(tree.get(&U16Vec3::new(15, 15, 15)), Some(7));
	}

	#[test]
	fn remove_area_clears_bulk_region_without_touching_neighbours() {
		let mut tree = GridTree::<PackedCell, U16Coord>::new();
		tree.add_area(&U16Vec3::ZERO, IVec3::splat(64), 7);
		tree.remove_area(&U16Vec3::new(16, 16, 16), IVec3::splat(32));
		assert_eq!(tree.get(&U16Vec3::new(15, 16, 16)), Some(7));
		assert_eq!(tree.get(&U16Vec3::new(16, 16, 16)), None);
	}

	#[test]
	fn clear_sdf_only_removes_inside_shape() {
		let mut tree = GridTree::<PackedCell, U16Coord>::new();
		tree.add_area(&U16Vec3::ZERO, IVec3::splat(16), 3);
		let center = Vec3::splat(8.0);
		let radius = 5.0f32;
		let sdf = |p: Vec3| (p - center).length() - radius;
		tree.clear_sdf(Vec3::ZERO, Vec3::splat(16.0), &sdf, bevy::math::IVec2::splat(9), 6);
		assert_eq!(tree.get(&U16Vec3::new(8, 8, 8)), None);
	}
}
