use voxel_trees::grid_tree::{GridTree64, GridType};
use serde::{Deserialize, Serialize};

use crate::voxels::{VoxelRef, VoxelType, VoxelTypeId, VoxelTypeInfo};

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

impl<'a, T: VoxelType + 'a> From<&'a T> for VoxelRef<'a> {
	fn from(v: &'a T) -> Self { v.get_ref() }
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

pub type VoxelGridTree = GridTree64<VoxelGridType>;
