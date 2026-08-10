use voxel_gpu::shader_compiler::SlangStage;
use voxel_gpu::{SlangAssetEntry, SlangAssetFile, SlangShaderSettings};

pub const ROOT_SHADER_ASSET: &str = "embedded://voxel_marching_renderer/shaders/marching_cubes.slang";

pub fn asset_settings() -> SlangShaderSettings {
	SlangShaderSettings {
		files: vec![SlangAssetFile { asset_path: ROOT_SHADER_ASSET.into(), compile_path: "marching/marching_cubes.slang".into() }],
		base_dir: "marching".into(),
		include_dirs: vec!["marching".into()],
		entries: vec![
			SlangAssetEntry { source: "marching_cubes.slang".into(), entry: "vs_main".into(), stage: SlangStage::Vertex },
			SlangAssetEntry { source: "marching_cubes.slang".into(), entry: "fs_main".into(), stage: SlangStage::Fragment },
		],
		linkages: Vec::new(),
		create_bevy_shader: true,
	}
}
