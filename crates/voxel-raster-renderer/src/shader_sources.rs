use voxel_gpu::shader_compiler::{ShaderResult, SlangStage};
use voxel_gpu::{SlangAssetEntry, SlangAssetFile, SlangShaderSettings};

pub const ROOT_SHADER_ASSET: &str = "embedded://voxel_raster_renderer/shaders/raster_voxel.slang";

pub fn asset_settings() -> SlangShaderSettings {
	SlangShaderSettings {
		files: vec![SlangAssetFile {
			asset_path: ROOT_SHADER_ASSET.into(),
			compile_path: "raster/raster_voxel.slang".into(),
		}],
		base_dir: "raster".into(),
		include_dirs: vec!["raster".into()],
		entries: vec![
			SlangAssetEntry { source: "raster_voxel.slang".into(), entry: "vs_main".into(), stage: SlangStage::Vertex },
			SlangAssetEntry { source: "raster_voxel.slang".into(), entry: "fs_main".into(), stage: SlangStage::Fragment },
		],
		linkages: Vec::new(),
	}
}

pub fn from_compiled(shaders: &[String]) -> ShaderResult<String> {
	if shaders.len() != 2 {
		return Err(std::io::Error::other("expected two compiled voxel raster shader stages").into());
	}
	Ok(shaders.join("\n"))
}
