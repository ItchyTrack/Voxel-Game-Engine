use wesl::{FileResolver, Router, Wesl};

fn main() {
	let mut router = Router::new();
	router.mount_fallback_resolver(FileResolver::new("src/shaders"));
	router.mount_resolver("shared".parse().unwrap(), FileResolver::new("../voxel-gpu/src/shaders"));

	let local_compiler = Wesl::new("").set_custom_resolver(router);
	local_compiler.build_artifact(&"package::beam".parse().unwrap(), "beam");
	local_compiler.build_artifact(&"package::raycasting".parse().unwrap(), "raycasting");
	local_compiler.build_artifact(&"package::coloring_shader".parse().unwrap(), "coloring_shader");
}
