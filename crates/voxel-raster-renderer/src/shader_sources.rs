use voxel_gpu::shader_compiler::{ShaderResult, SlangEntry, SlangStage};

const ENTRIES: &[SlangEntry<'_>] = &[
	SlangEntry { source: "raster_voxel.slang", entry: "vs_main", stage: SlangStage::Vertex },
	SlangEntry { source: "raster_voxel.slang", entry: "fs_main", stage: SlangStage::Fragment },
];

pub fn embedded() -> ShaderResult<String> {
	let manifest_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
	let local_shader_dir = manifest_dir.join("src/shaders");
	one(voxel_gpu::shader_sources::compile_embedded(&local_shader_dir, &local_shader_dir, ENTRIES, &[])?)
}

#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
pub fn load_from_disk() -> ShaderResult<String> {
	let manifest_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
	let local_shader_dir = manifest_dir.join("src/shaders");
	one(voxel_gpu::shader_sources::compile_from_disk(&local_shader_dir, &local_shader_dir, ENTRIES, &[])?)
}

fn one(shaders: Vec<String>) -> ShaderResult<String> {
	if shaders.is_empty() {
		return Err(std::io::Error::other("raster Slang compiler returned no shader").into());
	}
	Ok(shaders.join("\n"))
}
