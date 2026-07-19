use voxel_content::{VoxMaterial, VoxMaterialMapper};
use voxel_data::voxels::{Voxel, VoxelType, VoxelTypeId, VoxelTypeInfo};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BasicVoxel {
	pub color: [u8; 4],
	pub mass: u32,
}

impl VoxelType for BasicVoxel {
	const TYPE_INFO: VoxelTypeInfo = VoxelTypeInfo { id: VoxelTypeId(1), size_bytes: 8 };

	fn into_voxel(self) -> Voxel {
		let mut bytes = [0u8; Self::TYPE_INFO.size_bytes as usize];
		bytes[..4].copy_from_slice(&self.color);
		bytes[4..8].copy_from_slice(&self.mass.to_le_bytes());
		Voxel::new(Self::TYPE_INFO.id, bytes)
	}

	fn from_voxel(voxel: &Voxel) -> Self {
		voxel.assert_type::<Self>();
		let bytes = voxel.bytes();
		let mut color = [0u8; 4];
		let mut mass = [0u8; 4];
		color.copy_from_slice(&bytes[..4]);
		mass.copy_from_slice(&bytes[4..8]);
		Self { color, mass: u32::from_le_bytes(mass) }
	}

	fn from_voxel_ref(voxel: &voxel_data::voxels::VoxelRef) -> Self {
		voxel.assert_type::<Self>();
		let bytes = voxel.bytes();
		let mut color = [0u8; 4];
		let mut mass = [0u8; 4];
		color.copy_from_slice(&bytes[..4]);
		mass.copy_from_slice(&bytes[4..8]);
		Self { color, mass: u32::from_le_bytes(mass) }
	}

	fn into_bytes(self, bytes: &mut [u8]) {
		bytes[..4].copy_from_slice(&self.color);
		bytes[4..8].copy_from_slice(&self.mass.to_le_bytes());
	}

	fn from_bytes(bytes: &[u8]) -> Self {
		let mut color = [0u8; 4];
		let mut mass = [0u8; 4];
		color.copy_from_slice(&bytes[..4]);
		mass.copy_from_slice(&bytes[4..8]);
		Self { color, mass: u32::from_le_bytes(mass) }
	}
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BasicVoxMaterialMapper;

impl VoxMaterialMapper for BasicVoxMaterialMapper {
	fn voxel_type_info(&self) -> VoxelTypeInfo {
		BasicVoxel::TYPE_INFO
	}

	fn voxel(&self, material: VoxMaterial) -> Voxel {
		BasicVoxel { color: material.color, mass: 100 }.into_voxel()
	}
}
