use std::path::{Path, PathBuf};

use wesl::{FileResolver, Router, Wesl};

pub fn voxel_shader_watch_roots(manifest_dir: &Path) -> [PathBuf; 2] {
	[
		manifest_dir.join("src/shaders"),
		manifest_dir.join("../voxel-gpu/src/shaders"),
	]
}

pub fn voxel_shader_compiler(local_shader_dir: &Path, shared_shader_dir: &Path) -> Wesl<Router> {
	let mut router = Router::new();
	router.mount_fallback_resolver(FileResolver::new(local_shader_dir));
	router.mount_resolver("shared".parse().expect("valid shared shader namespace"), FileResolver::new(shared_shader_dir));
	Wesl::new("").set_custom_resolver(router)
}
