use voxel_gpu::shader_codegen::{EmbeddedWeslModule, ShaderResult};

const ENTRY_MODULE: &str = "package::raster_voxel";
const LOCAL_MODULES: &[EmbeddedWeslModule] = voxel_gpu::embedded_wesl_modules![
	"package::raster_voxel" => "shaders/raster_voxel.wesl",
];

pub fn embedded() -> ShaderResult<String> {
	one(voxel_gpu::shader_sources::compile_embedded(LOCAL_MODULES, &[ENTRY_MODULE])?)
}

#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
pub fn load_from_disk() -> ShaderResult<String> {
	let manifest_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
	one(voxel_gpu::shader_sources::compile_from_disk(
		&manifest_dir.join("src/shaders"),
		&manifest_dir.join("../voxel-gpu/src/shaders"),
		&[ENTRY_MODULE],
	)?)
}

fn one(mut shaders: Vec<String>) -> ShaderResult<String> {
	shaders.pop().ok_or_else(|| std::io::Error::other("raster WESL compiler returned no shader").into())
}
