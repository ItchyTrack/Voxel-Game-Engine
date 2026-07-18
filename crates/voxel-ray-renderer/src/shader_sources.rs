use std::path::Path;

use voxel_gpu::shader_codegen::{EmbeddedWeslModule, ShaderResult};

const ENTRY_MODULES: &[&str] = &[
	"package::beam",
	"package::raycasting",
	"package::coloring_shader",
];

const LOCAL_MODULES: &[EmbeddedWeslModule] = voxel_gpu::embedded_wesl_modules![
	"package::beam" => "shaders/beam.wesl",
	"package::raycasting" => "shaders/raycasting.wesl",
	"package::coloring_shader" => "shaders/coloring_shader.wesl",
];

#[derive(Clone)]
pub struct VoxelShaderSources {
	pub beam: String,
	pub raycasting: String,
	pub coloring: String,
}

impl VoxelShaderSources {
	pub fn embedded() -> ShaderResult<Self> {
		Self::from_compiled(voxel_gpu::shader_sources::compile_embedded(LOCAL_MODULES, ENTRY_MODULES)?)
	}

	#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
	pub fn load_from_disk() -> ShaderResult<Self> {
		let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
		let local_shader_dir = manifest_dir.join("src/shaders");
		let shared_shader_dir = manifest_dir.join("../voxel-gpu/src/shaders");
		Self::from_compiled(voxel_gpu::shader_sources::compile_from_disk(
			&local_shader_dir,
			&shared_shader_dir,
			ENTRY_MODULES,
		)?)
	}

	fn from_compiled(mut shaders: Vec<String>) -> ShaderResult<Self> {
		if shaders.len() != 3 {
			return Err(std::io::Error::other("expected three compiled voxel ray shaders").into());
		}
		let coloring = shaders.pop().unwrap();
		let raycasting = shaders.pop().unwrap();
		let beam = shaders.pop().unwrap();
		Ok(Self { beam, raycasting, coloring })
	}
}
