use voxel_content::{VoxMaterial, VoxMaterialVoxel};
use voxel_data::voxels::{VoxelType, VoxelTypeId};
use voxel_gpu::VoxelGpuData;
use voxel_physics::VoxelMassValue;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, bytemuck::Pod, bytemuck::Zeroable)]
pub struct BasicVoxel {
	pub color: [u8; 4],
	pub mass: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LodVoxel {
	pub colors: [[u8; 4]; 8],
}

impl LodVoxel {
	pub fn solid(color: [u8; 4]) -> Self {
		Self { colors: [color; 8] }
	}
}

impl VoxelType for BasicVoxel {
	const TYPE_ID: VoxelTypeId = VoxelTypeId(1);
}

impl VoxelType for LodVoxel {
	const TYPE_ID: VoxelTypeId = VoxelTypeId(2);
}

impl VoxMaterialVoxel for BasicVoxel {
	fn from_vox_material(material: VoxMaterial) -> Self {
		Self { color: material.color, mass: 100 }
	}
}

impl VoxelGpuData for BasicVoxel {
	fn shader_source() -> &'static str {
		concat!(env!("CARGO_MANIFEST_DIR"), "/src/shaders/basic_voxel.slang")
	}

	fn shader_sampler() -> &'static str {
		"BasicVoxelSampler"
	}

	fn voxel_gpu_size_bytes() -> usize { 4 }

	fn write_voxel_gpu_raw(&self, bytes: &mut [u8]) {
		bytes.copy_from_slice(&self.color);
	}
}

impl VoxelGpuData for LodVoxel {
	fn shader_source() -> &'static str {
		concat!(env!("CARGO_MANIFEST_DIR"), "/src/shaders/lod_voxel.slang")
	}

	fn shader_sampler() -> &'static str {
		"LodVoxelSampler"
	}

	fn voxel_gpu_size_bytes() -> usize { 32 }

	fn write_voxel_gpu_raw(&self, bytes: &mut [u8]) {
		bytes.copy_from_slice(bytemuck::bytes_of(&self.colors));
	}
}

impl VoxelMassValue for BasicVoxel {
	fn voxel_mass(&self) -> f64 {
		self.mass as f64
	}
}
