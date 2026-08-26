use bevy::math::Mat4;
use bevy::render::render_resource::ShaderType;
use voxel_data::voxels::VoxelTypeId;

#[derive(Debug, Copy, Clone, Default, ShaderType)]
pub struct ModelUniform {
	model: Mat4,
	voxel_type_id: u32,
	voxel_data_offset: u32,
}

impl ModelUniform {
	pub fn new(model: Mat4, voxel_type: VoxelTypeId, voxel_data_offset: u32) -> Self {
		Self { model, voxel_type_id: u32::from(voxel_type.0), voxel_data_offset }
	}
}
