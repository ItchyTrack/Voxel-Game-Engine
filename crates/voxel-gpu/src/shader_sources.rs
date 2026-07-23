use std::path::{Path, PathBuf};

use crate::shader_compiler::{self, ShaderResult, SlangEntry, SlangLinkage};

pub fn compile_from_disk(local_shader_dir: &Path, shared_shader_dir: &Path, entries: &[SlangEntry<'_>], linkages: &[SlangLinkage]) -> ShaderResult<Vec<String>> {
	let include_dirs = vec![
		local_shader_dir.to_path_buf(),
		shared_shader_dir.to_path_buf(),
	];
	shader_compiler::compile_slang_files(local_shader_dir, &include_dirs, entries, linkages)
}

pub fn compile_embedded(local_shader_dir: &Path, shared_shader_dir: &Path, entries: &[SlangEntry<'_>], linkages: &[SlangLinkage]) -> ShaderResult<Vec<String>> {
	// Slang is compiled by slangc for both hot-reload and normal builds. Release and wasm builds
	// should pre-generate/embed WGSL later if a no-toolchain runtime is required.
	compile_from_disk(local_shader_dir, shared_shader_dir, entries, linkages)
}

pub fn shared_shader_dir_from_manifest(manifest_dir: &Path) -> PathBuf {
	manifest_dir.join("../voxel-gpu/src/shaders")
}
