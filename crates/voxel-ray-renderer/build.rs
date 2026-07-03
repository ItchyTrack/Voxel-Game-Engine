#[path = "src/shader_common.rs"]
mod shader_common;

use std::path::{Path, PathBuf};

use wesl::emit_rerun_if_changed;

fn main() {
	let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
	let [local_shader_dir, shared_shader_dir] = shader_common::voxel_shader_watch_roots(manifest_dir);
	let compiler = shader_common::voxel_shader_compiler(&local_shader_dir, &shared_shader_dir);
	let output_dir = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR should be set"));

	for (module, artifact) in [
		("package::beam", "beam"),
		("package::raycasting", "raycasting"),
		("package::coloring_shader", "coloring_shader"),
	] {
		let compiled = compiler
			.compile(&module.parse().expect("valid shader module path"))
			.unwrap_or_else(|error| panic!("failed to build WESL shader `{module}`.\n{error}"));
		emit_rerun_if_changed(&compiled.modules, compiler.resolver());
		compiled
			.write_to_file(output_dir.join(format!("{artifact}.wgsl")))
			.unwrap_or_else(|error| panic!("failed to write shader artifact `{artifact}`: {error}"));
	}
}
