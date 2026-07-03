use std::error::Error;
use std::path::Path;

pub type ShaderSourceResult<T> = Result<T, Box<dyn Error + Send + Sync>>;

const BEAM_MODULE: &str = "package::beam";
const RAYCASTING_MODULE: &str = "package::raycasting";
const COLORING_MODULE: &str = "package::coloring_shader";

#[derive(Clone)]
pub struct VoxelShaderSources {
	pub beam: String,
	pub raycasting: String,
	pub coloring: String,
}

#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
pub fn compile_voxel_shader_sources(local_shader_dir: &Path, shared_shader_dir: &Path) -> ShaderSourceResult<VoxelShaderSources> {
	let compiler = crate::shader_common::voxel_shader_compiler(local_shader_dir, shared_shader_dir);

	Ok(VoxelShaderSources {
		beam: compiler.compile(&BEAM_MODULE.parse()?)?.to_string(),
		raycasting: compiler.compile(&RAYCASTING_MODULE.parse()?)?.to_string(),
		coloring: compiler.compile(&COLORING_MODULE.parse()?)?.to_string(),
	})
}
