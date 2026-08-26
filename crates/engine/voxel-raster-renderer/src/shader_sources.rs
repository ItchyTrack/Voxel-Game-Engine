use std::path::PathBuf;

use bevy::asset::{AssetServer, Handle};
use bevy::prelude::{DetectChanges, Res};
use voxel_gpu::shader_compiler::{SlangConformance, SlangLinkage, SlangModule, SlangStage};
use voxel_gpu::{
	CompiledSlangShader, SlangAssetEntry, SlangAssetFile, SlangShaderSettings,
	VOXEL_SAMPLER_API_ASSET, VoxelGpuShaderTypes,
};

const VOXEL_SHADER_INTERFACE: &str = "IVoxelSampler";
pub const ROOT_SHADER_ASSET: &str = "embedded://voxel_raster_renderer/shaders/raster_voxel.slang";

pub fn asset_settings(shader_types: &VoxelGpuShaderTypes) -> SlangShaderSettings {
	let shader_sources = shader_types.registrations();
	let mut files = vec![
		SlangAssetFile {
			asset_path: ROOT_SHADER_ASSET.into(),
			compile_path: "raster/raster_voxel.slang".into(),
		},
		SlangAssetFile {
			asset_path: VOXEL_SAMPLER_API_ASSET.into(),
			compile_path: "raster/voxel_sampler_api.slang".into(),
		},
	];
	files.extend(shader_sources.iter().map(|(type_id, source, _)| SlangAssetFile {
		asset_path: (*source).to_string(),
		compile_path: voxel_module_path(type_id.0),
	}));

	let voxel_linkage = SlangLinkage {
		modules: shader_sources.iter()
			.map(|(type_id, _, _)| SlangModule { path: voxel_module_path(type_id.0) })
			.collect(),
		conformances: shader_sources.iter()
			.map(|(type_id, _, sampler)| SlangConformance {
				ty: (*sampler).to_string(),
				interface: VOXEL_SHADER_INTERFACE.to_string(),
				id: type_id.0,
			})
			.collect(),
	};

	SlangShaderSettings {
		files,
		base_dir: "raster".into(),
		include_dirs: vec!["raster".into()],
		entries: vec![
			SlangAssetEntry { source: "raster_voxel.slang".into(), entry: "vs_main".into(), stage: SlangStage::Vertex },
			SlangAssetEntry { source: "raster_voxel.slang".into(), entry: "fs_main".into(), stage: SlangStage::Fragment },
		],
		linkages: vec![SlangLinkage::default(), voxel_linkage],
		create_bevy_shader: true,
	}
}

pub fn load_shader(
	asset_server: &AssetServer,
	shader_types: &VoxelGpuShaderTypes,
) -> Handle<CompiledSlangShader> {
	let shader_types = shader_types.clone();
	asset_server
		.load_builder()
		.with_settings(move |current: &mut SlangShaderSettings| {
			*current = asset_settings(&shader_types);
		})
		.load(ROOT_SHADER_ASSET)
}

pub fn reload_shader_on_type_change(
	asset_server: Res<AssetServer>,
	shader_types: Res<VoxelGpuShaderTypes>,
) {
	if !shader_types.is_changed() || shader_types.is_added() { return; }
	asset_server.reload(ROOT_SHADER_ASSET);
}

fn voxel_module_path(type_id: u16) -> PathBuf {
	PathBuf::from(format!("raster/voxel/{type_id}.slang"))
}
