use std::path::Path;

use wesl::{FileResolver, Router, VirtualResolver, Wesl};

use crate::shader_codegen::{
	self, EmbeddedWeslModule, GeneratedWeslModule, ShaderResult,
};

pub const SHARED_MODULES: &[EmbeddedWeslModule] = crate::embedded_wesl_modules![
	"shared::beam_combined_raycast" => "shaders/beam_combined_raycast.wesl",
	"shared::bvh::beam_raycast" => "shaders/bvh/beam_raycast.wesl",
	"shared::bvh::data" => "shaders/bvh/data.wesl",
	"shared::bvh::raycast" => "shaders/bvh/raycast.wesl",
	"shared::combined_raycast" => "shaders/combined_raycast.wesl",
	"shared::dda::data" => "shaders/dda/data.wesl",
	"shared::direction_feedback" => "shaders/direction_feedback.wesl",
	"shared::helpers::raycast" => "shaders/helpers/raycast.wesl",
];

const DATA_SOURCE_TEMPLATE: &str = include_str!("shaders/dda/data_source.template.wesl");
const RAYCAST_TEMPLATE: &str = include_str!("shaders/dda/raycast.template.wesl");
const BEAM_RAYCAST_TEMPLATE: &str = include_str!("shaders/dda/beam_raycast.template.wesl");
const VOXEL_READER_TEMPLATE: &str = include_str!("shaders/voxel_reader.template.wesl");

pub fn compile_embedded(local_modules: &[EmbeddedWeslModule], entry_modules: &[&str]) -> ShaderResult<Vec<String>> {
	let modules: Vec<_> = SHARED_MODULES.iter().chain(local_modules).copied().collect();
	let generated = generate_modules(DATA_SOURCE_TEMPLATE, RAYCAST_TEMPLATE, BEAM_RAYCAST_TEMPLATE, VOXEL_READER_TEMPLATE)?;
	shader_codegen::compile_embedded(&modules, &generated, entry_modules)
}

pub fn compile_from_disk(local_shader_dir: &Path, shared_shader_dir: &Path, entry_modules: &[&str]) -> ShaderResult<Vec<String>> {
	let data_source_template = std::fs::read_to_string(shared_shader_dir.join("dda/data_source.template.wesl"))?;
	let raycast_template = std::fs::read_to_string(shared_shader_dir.join("dda/raycast.template.wesl"))?;
	let beam_template = std::fs::read_to_string(shared_shader_dir.join("dda/beam_raycast.template.wesl"))?;
	let voxel_reader_template = std::fs::read_to_string(shared_shader_dir.join("voxel_reader.template.wesl"))?;
	let generated_modules = generate_modules(&data_source_template, &raycast_template, &beam_template, &voxel_reader_template)?;
	let mut generated = VirtualResolver::new();
	for module in generated_modules {
		generated.add_module(module.module.parse()?, module.source.into());
	}

	let mut router = Router::new();
	router.mount_fallback_resolver(FileResolver::new(local_shader_dir));
	router.mount_resolver("shared".parse()?, FileResolver::new(shared_shader_dir));
	router.mount_resolver("shared::generated".parse()?, generated);
	shader_codegen::compile(Wesl::new("").set_custom_resolver(router), entry_modules)
}

fn generate_modules(data_source_template: &str, raycast_template: &str, beam_template: &str, voxel_reader_template: &str) -> ShaderResult<Vec<GeneratedWeslModule>> {
	Ok(vec![
		GeneratedWeslModule {
			module: "shared::generated::dda::data_residency".to_owned(),
			source: shader_codegen::specialize(data_source_template, &[
				("BINDING", "0"),
				("BUFFER", "residency_grid_tree_buf"),
				("PREFIX", "residency"),
			])?,
		},
		GeneratedWeslModule {
			module: "shared::generated::dda::data_main".to_owned(),
			source: shader_codegen::specialize(data_source_template, &[
				("BINDING", "1"),
				("BUFFER", "main_grid_tree_buf"),
				("PREFIX", "main"),
			])?,
		},
		GeneratedWeslModule {
			module: "shared::generated::voxel_reader_residency".to_owned(),
			source: shader_codegen::specialize(voxel_reader_template, &[
				("BINDING", "0"),
				("BUFFER", "residency_voxel_data_buf"),
				("PREFIX", "residency"),
			])?,
		},
		GeneratedWeslModule {
			module: "shared::generated::voxel_reader_main".to_owned(),
			source: shader_codegen::specialize(voxel_reader_template, &[
				("BINDING", "1"),
				("BUFFER", "main_voxel_data_buf"),
				("PREFIX", "main"),
			])?,
		},
		GeneratedWeslModule {
			module: "shared::generated::dda::raycast_residency".to_owned(),
			source: specialize_raycast(raycast_template, "residency")?,
		},
		GeneratedWeslModule {
			module: "shared::generated::dda::raycast_main".to_owned(),
			source: specialize_raycast(raycast_template, "main")?,
		},
		GeneratedWeslModule {
			module: "shared::generated::dda::beam_raycast_residency".to_owned(),
			source: specialize_beam(beam_template, "residency")?,
		},
		GeneratedWeslModule {
			module: "shared::generated::dda::beam_raycast_main".to_owned(),
			source: specialize_beam(beam_template, "main")?,
		},
	])
}

fn specialize_raycast(template: &str, prefix: &str) -> ShaderResult<String> {
	let raycast = format!("dda_raycast_{prefix}");
	let u32_reader = format!("{prefix}_dda_u32");
	let parent_offset = format!("{prefix}_dda_parent_offset");
	let voxel_data_offset = format!("{prefix}_dda_voxel_data_offset");
	let node_entry = format!("{prefix}_dda_node_entry");
	let data_module = format!("data_{prefix}");
	shader_codegen::specialize(template, &[
		("DATA_MODULE", &data_module),
		("DDA_RAYCAST", &raycast),
		("DDA_U32", &u32_reader),
		("DDA_PARENT_OFFSET", &parent_offset),
		("DDA_VOXEL_DATA_OFFSET", &voxel_data_offset),
		("DDA_NODE_ENTRY", &node_entry),
	])
}

fn specialize_beam(template: &str, prefix: &str) -> ShaderResult<String> {
	let raycast = format!("beam_dda_raycast_{prefix}");
	let u32_reader = format!("{prefix}_dda_u32");
	let parent_offset = format!("{prefix}_dda_parent_offset");
	let node_entry = format!("{prefix}_dda_node_entry");
	let data_module = format!("data_{prefix}");
	shader_codegen::specialize(template, &[
		("DATA_MODULE", &data_module),
		("BEAM_DDA_RAYCAST", &raycast),
		("DDA_U32", &u32_reader),
		("DDA_PARENT_OFFSET", &parent_offset),
		("DDA_NODE_ENTRY", &node_entry),
	])
}
