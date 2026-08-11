pub mod packed_dynamic_buffer;
pub mod packed_residency_buffer;
pub mod packed_buffer_group;
pub mod rendering_generator;
pub mod residency_packing;
pub mod shader_compiler;
pub mod slang_shader_asset;
pub mod voxel_color;

use bevy::{
	ecs::message::message_update_system,
	prelude::*,
	render::{ExtractSchedule, Render, RenderApp, RenderSystems, extract_resource::ExtractResourcePlugin, render_asset::RenderAssetPlugin},
};

pub use packed_dynamic_buffer::{AllocationId, PackedBufferAllocation};
pub use rendering_generator::{
	RenderingContext, RenderingGenerationPlugin, RenderingGeneratorAppExt,
	RenderingGeneratorRegistry, RenderingTileClass, RenderingTileGenerator, RenderingType,
};
pub use slang_shader_asset::{
	CompiledSlangShader, LoadedSlangShader, SlangAssetEntry, SlangAssetFile, SlangShader,
	SlangShaderChanged, SlangShaderLoader, SlangShaderParam, SlangShaderSettings,
};
pub use voxel_color::{
	VoxelGpuAppExt, VoxelGpuBlockEncoder, VoxelGpuData, VoxelGpuDataReaders,
	VoxelGpuNodeEntry, VoxelGpuShaderTypes, VoxelShaderRegistration,
};

/// Shared GPU helpers and voxel shader-data registration. Renderer-specific
/// allocations, tile data, and upload paths live in the renderer crates.
#[derive(Default)]
pub struct GpuVoxelDataPlugin;

impl Plugin for GpuVoxelDataPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<VoxelGpuDataReaders>()
			.init_resource::<VoxelGpuShaderTypes>()
			.init_asset::<CompiledSlangShader>()
			.init_asset_loader::<SlangShaderLoader>()
			.add_plugins((
				ExtractResourcePlugin::<VoxelGpuDataReaders>::default(),
				RenderAssetPlugin::<CompiledSlangShader>::default(),
			));
		if let Some(render_app) = app.get_sub_app_mut(RenderApp) {
			render_app
				.add_message::<SlangShaderChanged>()
				.add_systems(ExtractSchedule, slang_shader_asset::extract_shader_changes)
				.add_systems(Render, message_update_system.in_set(RenderSystems::PostCleanup));
		}
	}
}
