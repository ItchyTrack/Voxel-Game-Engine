use crate::{grid_tree::{AsGridData, GridTree, GridType, U16Coord}, voxels::{VoxelRef, VoxelType, VoxelTypeId, VoxelTypeInfo}};
use serde::{Deserialize, Serialize};

/// Fixed-width `u16` grid type for non-voxel trees that store compact ids.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Serialize, Deserialize)]
pub struct PackedCell;

impl GridType for PackedCell {
	type Data<'a> = u16;
	const MAX_NODE_OFFSET: u32 = u32::MAX;
	fn data_size_bytes(&self) -> usize { std::mem::size_of::<u16>() }
	fn read_data<'a>(&self, bytes: &'a [u8]) -> Self::Data<'a> {
		u16::from_le_bytes(bytes[..2].try_into().expect("PackedCell data bytes"))
	}
	fn write_data(&self, data: Self::Data<'_>, bytes: &mut [u8]) {
		bytes[..2].copy_from_slice(&data.to_le_bytes());
	}
	fn data_eq_bytes(&self, data: Self::Data<'_>, bytes: &[u8]) -> bool {
		bytes[..2] == data.to_le_bytes()
	}
}

/// Voxel grid storage type. The tree stores voxel bytes directly in each data slot.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct VoxelGridType {
	type_info: VoxelTypeInfo,
}

impl VoxelGridType {
	pub fn new(type_info: VoxelTypeInfo) -> Self { Self { type_info } }
	pub fn type_info(self) -> VoxelTypeInfo { self.type_info }
	pub fn type_id(self) -> VoxelTypeId { self.type_info.id }
}

impl<'a, T: VoxelType + 'a> AsGridData<'a, VoxelGridType> for &'a T {
	fn as_grid_data(self) -> VoxelRef<'a> { self.get_ref() }
}

impl GridType for VoxelGridType {
	type Data<'a> = VoxelRef<'a>;
	const MAX_NODE_OFFSET: u32 = u32::MAX;
	fn data_size_bytes(&self) -> usize { self.type_info.size_bytes as usize }
	fn read_data<'a>(&self, bytes: &'a [u8]) -> Self::Data<'a> {
		VoxelRef::new(self.type_info.id, &bytes[..self.data_size_bytes()])
	}
	fn write_data(&self, data: Self::Data<'_>, bytes: &mut [u8]) {
		self.type_info.id.assert_type(data.type_id());
		bytes[..self.data_size_bytes()].copy_from_slice(data.bytes());
	}
	fn data_eq_bytes(&self, data: Self::Data<'_>, bytes: &[u8]) -> bool {
		data.type_id() == self.type_info.id && data.bytes() == &bytes[..self.data_size_bytes()]
	}
}

pub type PackedGridTree = GridTree<PackedCell, U16Coord>;
pub type VoxelGridTree = GridTree<VoxelGridType, U16Coord>;
pub type PackedNode<'a> = crate::grid_tree::GridTreeNode<'a>;
