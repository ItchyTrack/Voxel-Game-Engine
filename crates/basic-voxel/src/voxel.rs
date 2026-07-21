use voxel_content::{VoxMaterial, VoxMaterialVoxel};
use voxel_data::voxels::{VoxelType, VoxelTypeId};
use voxel_gpu::VoxelColor;
use voxel_physics::VoxelMassValue;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, bytemuck::Pod, bytemuck::Zeroable)]
pub struct BasicVoxel {
	pub color: [u8; 4],
	pub mass: u32,
}

impl VoxelType for BasicVoxel {
	const TYPE_ID: VoxelTypeId = VoxelTypeId(1);
}

impl VoxMaterialVoxel for BasicVoxel {
	fn from_vox_material(material: VoxMaterial) -> Self {
		Self { color: material.color, mass: 100 }
	}
}

impl VoxelColor for BasicVoxel {
	fn voxel_color(&self) -> [u8; 4] {
		self.color
	}
}

impl VoxelMassValue for BasicVoxel {
	fn voxel_mass(&self) -> f64 {
		self.mass as f64
	}
}
