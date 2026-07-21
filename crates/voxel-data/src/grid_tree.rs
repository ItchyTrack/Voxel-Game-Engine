use std::marker::PhantomData;

use bevy::math::{I8Vec3, IVec3, U8Vec3};
use bevy::transform::components::Transform;

mod cell;
mod coord;
mod data;
mod raw;
mod raycast;
mod traversal;
mod view;
pub use cell::CellKind;
pub use coord::{GridCoord, U16Coord, U32Coord};
pub use data::{GridData, GridType};
pub use raw::GridTreeNode;
pub use view::{CellRef, ChildCells, ChildCellsInRegion, GridTreeView, LeafCells, NodeRef};

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

pub fn get_child_contents_index(contents_pos: U8Vec3) -> u8 {
	contents_pos.x + contents_pos.y * SIZE + contents_pos.z * SIZE * SIZE
}

pub fn get_child_contents_pos(contents_index: u8) -> U8Vec3 {
	U8Vec3::new(contents_index % SIZE, (contents_index / SIZE) % SIZE, contents_index / (SIZE * SIZE))
}

#[derive(Debug, Clone)]
pub struct GridTree<G: GridType, Co: GridCoord> {
	grid_type: G,
	raw: raw::RawGridTree,
	_coord: PhantomData<Co>,
}

#[derive(Debug)]
struct AreaOp<'a, G: GridType> {
	min: IVec3,
	end: IVec3,
	data: G::Data<'a>,
}

impl<'a, G: GridType> Copy for AreaOp<'a, G> {}
impl<'a, G: GridType> Clone for AreaOp<'a, G> { fn clone(&self) -> Self { *self } }

#[derive(Debug)]
enum CellWrite<'a, G: GridType> {
	Empty,
	Data(G::Data<'a>),
}

impl<'a, G: GridType> Copy for CellWrite<'a, G> {}
impl<'a, G: GridType> Clone for CellWrite<'a, G> { fn clone(&self) -> Self { *self } }

impl<G: GridType + Default, Co: GridCoord> GridTree<G, Co> {
	pub fn new() -> Self {
		Self::new_with_type(G::default())
	}
}

impl<G: GridType, Co: GridCoord> GridTree<G, Co> {
	pub fn new_with_type(grid_type: G) -> Self {
		let raw = raw::RawGridTree::new(grid_type.data_size_bytes());
		Self { grid_type, raw, _coord: PhantomData }
	}

	pub fn grid_type(&self) -> &G { &self.grid_type }

	pub fn view(&self) -> GridTreeView<'_, G, Co> {
		GridTreeView::new(&self.grid_type, &self.raw)
	}

	pub fn get(&self, pos: &Co::Pos) -> Option<G::Data<'_>> {
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

	#[inline]
	pub(crate) fn cell_kind(&self, node_index: u32, child_index: u8) -> CellKind {
		self.raw.cell_kind(node_index, child_index)
	}

	#[inline]
	pub(crate) fn child_index(&self, node_index: u32, child_index: u8) -> u32 {
		self.raw.child_index(node_index, child_index)
	}

	#[inline]
	pub(crate) fn cell_data(&self, node_index: u32, child_index: u8) -> G::Data<'_> {
		self.grid_type.read_data(self.raw.cell_bytes(node_index, child_index))
	}

	#[inline]
	pub(crate) fn cell_data_eq(&self, node_index: u32, child_index: u8, data: G::Data<'_>) -> bool {
		self.grid_type.data_eq_bytes(data, self.raw.cell_bytes(node_index, child_index))
	}

	#[inline]
	pub(crate) fn data_cells_eq(&self, first_node: u32, first_child: u8, second_node: u32, second_child: u8) -> bool {
		let data_size = self.grid_type.data_size_bytes();
		self.raw.cell_bytes(first_node, first_child)[..data_size] == self.raw.cell_bytes(second_node, second_child)[..data_size]
	}
}

mod serialize;
mod bulk;
mod point_ops;
mod region;
mod region_ops;
mod reduce;
mod single_build;
mod storage;
mod surgery;

pub use region::GridRegion;
pub use reduce::{reduce_grid_trees, SourceOverlap, SourceOverlaps, SourceTree};

impl<G: GridType, Co: GridCoord> GridTree<G, Co> {
	pub fn len(&self) -> u64 {
		self.raw.item_count()
	}
	pub fn is_empty(&self) -> bool {
		self.raw.is_empty()
	}
	pub fn iter(&self) -> GridTreeIterator<'_, G, Co> {
		GridTreeIterator::new(self)
	}

	/// Visit every DATA leaf whose cell box intersects a half-open region.
	pub fn for_each_in_region(&self, region: GridRegion, f: impl FnMut(Co::Pos, Co::Size, G::Data<'_>)) {
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

impl<'a, G: GridType, Co: GridCoord> IntoIterator for &'a GridTree<G, Co> {
	type Item = (Co::Pos, Co::Size, G::Data<'a>);
	type IntoIter = GridTreeIterator<'a, G, Co>;

	fn into_iter(self) -> Self::IntoIter {
		self.iter()
	}
}

pub struct GridTreeIterator<'a, G: GridType, Co: GridCoord> {
	leaves: LeafCells<'a, G, Co>,
}

impl<'a, G: GridType, Co: GridCoord> GridTreeIterator<'a, G, Co> {
	pub fn new(tree: &'a GridTree<G, Co>) -> Self {
		Self { leaves: tree.view().leaves() }
	}
}

impl<'a, G: GridType, Co: GridCoord> Iterator for GridTreeIterator<'a, G, Co> {
	type Item = (Co::Pos, Co::Size, G::Data<'a>);

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
