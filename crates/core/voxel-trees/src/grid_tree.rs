use bevy::math::{IVec3, U8Vec3, UVec3};

mod data;
mod raw;
mod view;
pub use crate::views::CellKind;
pub use data::{GridData, GridType};
pub use raw::GridTreeNode;
use serde::{Deserialize, Serialize};
pub use view::GridTreeViewImpl;
use crate::views::LeafCells;

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
pub struct GridTree<G: GridType> {
	grid_type: G,
	raw: raw::RawGridTree,
}

#[derive(Debug)]
struct AreaOp<'a, G: GridType> {
	region: NonZeroVoxelRegion,
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

impl<G: GridType + Default> GridTree<G> {
	pub fn new() -> Self {
		Self::new_with_type(G::default())
	}
}

impl<G: GridType> GridTree<G> {
	pub fn new_with_type(grid_type: G) -> Self {
		let raw = raw::RawGridTree::new(grid_type.data_size_bytes());
		Self { grid_type, raw }
	}

	pub fn grid_type(&self) -> &G { &self.grid_type }

	pub fn view(&self) -> GridTreeViewImpl<'_, G> {
		GridTreeViewImpl::new(self.grid_type, &self.raw)
	}

	pub fn contains_key(&self, pos: UVec3) -> bool {
		self.get(pos).is_some()
	}

	pub fn is_region_filled(&self, region: NonZeroVoxelRegion) -> bool {
		self.view().is_region_filled(region)
	}

	pub fn any_in_region(&self, region: NonZeroVoxelRegion) -> bool {
		self.view().any_in_region(region)
	}

	pub fn get(&self, pos: UVec3) -> Option<G::Data<'_>> {
		self.view().get(pos)
	}

	pub fn ensure_area_covered(&mut self, pos: &UVec3, size: UVec3) -> bool {
		if size.cmple(UVec3::ZERO).any() {
			return true;
		}
		let min = pos;
		let max = min + size - UVec3::ONE;
		self.make_sure_root_covers_area(*min, max)
	}

	pub fn for_each_in_region<F>(&self, region: NonZeroVoxelRegion, f: F)
	where
		F: FnMut(UVec3, G::Data<'_>),
	{
		self.view().for_each_in_region(region, f);
	}

	pub fn for_each_leaf_in_region<F>(&self, region: NonZeroVoxelRegion, f: F)
	where
		F: FnMut(UVec3, u32, G::Data<'_>),
	{
		self.view().for_each_leaf_in_region(region, f);
	}

	pub fn for_each_node_in_region<F>(&self, region: NonZeroVoxelRegion, f: F)
	where
		F: FnMut(UVec3, u32, bool),
	{
		self.view().for_each_node_in_region(region.min().as_uvec3(), region.max().as_uvec3(), f);
	}

	pub fn for_each_occupied_tile_cover<F>(
		&self,
		region: NonZeroVoxelRegion,
		tile_size: u32,
		f: F,
	) where
		F: FnMut(UVec3),
	{
		self.view().for_each_occupied_tile_cover(region.min().as_uvec3(), region.max().as_uvec3(), tile_size, f);
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
mod region_ops;
mod reduce;
mod single_build;
mod storage;
mod surgery;

pub use crate::region::NonZeroVoxelRegion;
use crate::views::{GridTreeView, GridView};
pub use reduce::{reduce_grid_trees, GridReducer, SourceOverlap, SourceOverlaps, SourceTree};

impl<G: GridType> GridTree<G> {
	pub fn len(&self) -> u64 {
		self.raw.item_count()
	}
	pub fn is_empty(&self) -> bool {
		self.raw.is_empty()
	}
	pub fn iter(&self) -> GridTreeIterator<'_, G> {
		GridTreeIterator::new(self)
	}
}

impl<'a, G: GridType> IntoIterator for &'a GridTree<G> {
	type Item = (UVec3, u32, G::Data<'a>);
	type IntoIter = GridTreeIterator<'a, G>;

	fn into_iter(self) -> Self::IntoIter {
		self.iter()
	}
}

pub struct GridTreeIterator<'a, G: GridType> {
	leaves: LeafCells<'a, GridTreeViewImpl<'a, G>>,
}

impl<'a, G: GridType> GridTreeIterator<'a, G> {
	pub fn new(tree: &'a GridTree<G>) -> Self {
		Self { leaves: tree.view().leaves() }
	}
}

impl<'a, G: GridType> Iterator for GridTreeIterator<'a, G> {
	type Item = (UVec3, u32, G::Data<'a>);

	fn next(&mut self) -> Option<Self::Item> {
		self.leaves.next().and_then(|leaf| Some((leaf.origin, leaf.size(), self.leaves.get_data(&leaf)?)))
	}
}

/// Fixed-width `u16` grid type
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Serialize, Deserialize)]
pub struct U16Cell;

impl GridType for U16Cell {
	type Data<'a> = u16;
	const MAX_NODE_OFFSET: u32 = u32::MAX;
	fn data_size_bytes(&self) -> usize { std::mem::size_of::<u16>() }
	fn read_data<'a>(&self, bytes: &'a [u8]) -> Self::Data<'a> {
		u16::from_le_bytes(bytes[..2].try_into().expect("U16Cell data bytes"))
	}
	fn write_data(&self, data: Self::Data<'_>, bytes: &mut [u8]) {
		bytes[..2].copy_from_slice(&data.to_le_bytes());
	}
	fn data_eq_bytes(&self, data: Self::Data<'_>, bytes: &[u8]) -> bool {
		bytes[..2] == data.to_le_bytes()
	}
}

/// Fixed-width `u32` grid type
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Serialize, Deserialize)]
pub struct U32Cell;

impl GridType for U32Cell {
	type Data<'a> = u32;
	const MAX_NODE_OFFSET: u32 = u32::MAX;
	fn data_size_bytes(&self) -> usize { std::mem::size_of::<u32>() }
	fn read_data<'a>(&self, bytes: &'a [u8]) -> Self::Data<'a> {
		u32::from_le_bytes(bytes[..4].try_into().expect("U32Cell data bytes"))
	}
	fn write_data(&self, data: Self::Data<'_>, bytes: &mut [u8]) {
		bytes[..4].copy_from_slice(&data.to_le_bytes());
	}
	fn data_eq_bytes(&self, data: Self::Data<'_>, bytes: &[u8]) -> bool {
		bytes[..4] == data.to_le_bytes()
	}
}

/// Fixed-width `u64` grid type
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Serialize, Deserialize)]
pub struct U64Cell;

impl GridType for U64Cell {
	type Data<'a> = u64;
	const MAX_NODE_OFFSET: u32 = u32::MAX;
	fn data_size_bytes(&self) -> usize { std::mem::size_of::<u64>() }
	fn read_data<'a>(&self, bytes: &'a [u8]) -> Self::Data<'a> {
		u64::from_le_bytes(bytes[..8].try_into().expect("U64Cell data bytes"))
	}
	fn write_data(&self, data: Self::Data<'_>, bytes: &mut [u8]) {
		bytes[..8].copy_from_slice(&data.to_le_bytes());
	}
	fn data_eq_bytes(&self, data: Self::Data<'_>, bytes: &[u8]) -> bool {
		bytes[..8] == data.to_le_bytes()
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use bevy::math::Vec3;

	#[test]
	fn add_area_preserves_large_runs() {
		let mut tree = GridTree::<U16Cell>::new();
		tree.add_area(&UVec3::ZERO, UVec3::splat(16), 7);
		assert_eq!(tree.len(), 16 * 16 * 16);
		assert_eq!(tree.get(UVec3::new(0, 0, 0)), Some(7));
		assert_eq!(tree.get(UVec3::new(15, 15, 15)), Some(7));
	}

	#[test]
	fn remove_area_clears_bulk_region_without_touching_neighbours() {
		let mut tree = GridTree::<U16Cell>::new();
		tree.add_area(&UVec3::ZERO, UVec3::splat(64), 7);
		tree.remove_area(&UVec3::new(16, 16, 16), UVec3::splat(32));
		assert_eq!(tree.get(UVec3::new(15, 16, 16)), Some(7));
		assert_eq!(tree.get(UVec3::new(16, 16, 16)), None);
	}

	#[test]
	fn clear_sdf_only_removes_inside_shape() {
		let mut tree = GridTree::<U16Cell>::new();
		tree.add_area(&UVec3::ZERO, UVec3::splat(16), 3);
		let center = Vec3::splat(8.0);
		let radius = 5.0f32;
		let sdf = |p: Vec3| (p - center).length() - radius;
		tree.clear_sdf(Vec3::ZERO, Vec3::splat(16.0), &sdf, bevy::math::IVec2::splat(9), 6);
		assert_eq!(tree.get(UVec3::new(8, 8, 8)), None);
	}
}
