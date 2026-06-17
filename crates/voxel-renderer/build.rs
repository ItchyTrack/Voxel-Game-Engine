use wesl::Wesl;

fn main() {
	let compiler = Wesl::new("../voxel-gpu/src/shaders");

	compiler.build_artifact(&"package::beam_bvh_raycast".parse().unwrap(), "beam_bvh_raycast");
	compiler.build_artifact(&"package::raycasting_shader".parse().unwrap(), "raycasting_shader");
	compiler.build_artifact(&"package::coloring_shader".parse().unwrap(), "coloring_shader");
}
