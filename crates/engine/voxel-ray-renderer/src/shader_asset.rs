use bevy::asset::{AssetServer, Handle};
use bevy::prelude::{DetectChanges, Res};
use voxel_gpu::{CompiledSlangShader, SlangShaderSettings, VoxelGpuShaderTypes};

pub fn load_voxel_ray_shader(
	asset_server: &AssetServer,
	shader_types: &VoxelGpuShaderTypes,
) -> Handle<CompiledSlangShader> {
	let shader_types = shader_types.clone();
	asset_server
		.load_builder()
		.with_settings(move |current: &mut SlangShaderSettings| {
			*current = crate::shader_sources::asset_settings(&shader_types);
		})
		.load(crate::shader_sources::ROOT_SHADER_ASSET)
}

pub fn reload_voxel_ray_shader_on_type_change(
	asset_server: Res<AssetServer>,
	shader_types: Res<VoxelGpuShaderTypes>,
) {
	if !shader_types.is_changed() || shader_types.is_added() { return; }
	asset_server.reload(crate::shader_sources::ROOT_SHADER_ASSET);
}
