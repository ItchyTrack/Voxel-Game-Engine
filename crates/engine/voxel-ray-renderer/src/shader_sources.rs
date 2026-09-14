use std::path::PathBuf;

use voxel_gpu::shader_compiler::{ShaderResult, SlangConformance, SlangLinkage, SlangModule, SlangStage};
use voxel_gpu::{SlangAssetEntry, SlangAssetFile, SlangShaderSettings, VoxelGpuShaderTypes};

const VOXEL_SHADER_INTERFACE: &str = "IVoxelSampler";
pub const ROOT_SHADER_ASSET: &str = "embedded://voxel_ray_renderer/shaders/beam.slang";
const RAY_SHADER_ROOT: &str = "embedded://voxel_ray_renderer/shaders/";

const LOCAL_FILES: &[&str] = &[
	"beam.slang",
	"raycasting.slang",
	"face_gi.slang",
	"face_gi_common.slang",
	"coloring_shader.slang",
	"coloring_common.slang",
	"anti_aliasing.slang",
];

const SHARED_FILES: &[&str] = &[
	"beam_combined_raycast.slang",
	"bvh/beam_raycast.slang",
	"bvh/data.slang",
	"bvh/raycast.slang",
	"combined_raycast.slang",
	"occlusion_raycast.slang",
	"dda/beam_raycast.slang",
	"dda/data.slang",
	"dda/raycast.slang",
	"direction_feedback.slang",
	"helpers/aabb.slang",
	"helpers/quat.slang",
	"voxel_reader.slang",
];

#[derive(Clone, PartialEq)]
pub struct VoxelShaderSources {
	pub beam: String,
	pub raycasting: String,
	pub coloring: String,
	pub fallback: String,
	pub face_gi: [String; 7],
}

impl VoxelShaderSources {
	pub fn from_compiled(shaders: &[String]) -> ShaderResult<Self> {
		if shaders.len() != 12 {
			return Err(std::io::Error::other("expected twelve compiled voxel ray shader stages").into());
		}
		Ok(Self {
			beam: shaders[0].clone(),
			raycasting: shaders[1].clone(),
			coloring: format!("{}\n{}", shaders[2], shaders[3]),
			fallback: shaders[4].clone(),
			face_gi: std::array::from_fn(|i| shaders[5 + i].clone()),
		})
	}
}

pub fn asset_settings(shader_types: &VoxelGpuShaderTypes) -> SlangShaderSettings {
	let mut files = LOCAL_FILES.iter()
		.map(|path| SlangAssetFile {
			asset_path: format!("{RAY_SHADER_ROOT}{path}"),
			compile_path: PathBuf::from("ray").join(path),
		})
		.chain(SHARED_FILES.iter().map(|path| SlangAssetFile {
			asset_path: format!("{RAY_SHADER_ROOT}shared/{path}"),
			compile_path: PathBuf::from("ray/shared").join(path),
		}))
		.chain(std::iter::once(SlangAssetFile {
			asset_path: voxel_gpu::VOXEL_SAMPLER_API_ASSET.into(),
			compile_path: "ray/shared/voxel_sampler_api.slang".into(),
		}))
		.collect::<Vec<_>>();
	let shader_sources = shader_types.registrations();
	files.extend(shader_sources.iter().map(|(type_id, source, _)| SlangAssetFile {
		asset_path: (*source).to_string(),
		compile_path: voxel_module_path(type_id.0),
	}));

	let voxel_linkage = SlangLinkage {
		modules: shader_sources.iter()
			.map(|(type_id, _, _)| SlangModule { path: voxel_module_path(type_id.0) })
			.collect(),
		conformances: shader_sources.iter()
			.map(|(type_id, _, sampler)| SlangConformance {
				ty: (*sampler).to_string(),
				interface: VOXEL_SHADER_INTERFACE.to_string(),
				id: type_id.0,
			})
			.collect(),
	};

	let mut entries = vec![
		SlangAssetEntry { source: "beam.slang".into(), entry: "main".into(), stage: SlangStage::Compute },
		SlangAssetEntry { source: "raycasting.slang".into(), entry: "main".into(), stage: SlangStage::Compute },
		SlangAssetEntry { source: "coloring_shader.slang".into(), entry: "vs_main".into(), stage: SlangStage::Vertex },
		SlangAssetEntry { source: "coloring_shader.slang".into(), entry: "fs_main".into(), stage: SlangStage::Fragment },
		SlangAssetEntry { source: "raycasting.slang".into(), entry: "fallback_main".into(), stage: SlangStage::Compute },
	];
	entries.extend(crate::face_gi::GI_ENTRIES.iter().map(|entry| SlangAssetEntry {
		source: "face_gi.slang".into(), entry: (*entry).into(), stage: SlangStage::Compute,
	}));
	let mut linkages = vec![
		SlangLinkage::default(), SlangLinkage::default(),
		voxel_linkage.clone(), voxel_linkage.clone(), SlangLinkage::default(),
	];
	linkages.extend(crate::face_gi::GI_ENTRIES.iter().map(|_| voxel_linkage.clone()));

	SlangShaderSettings {
		files,
		base_dir: "ray".into(),
		include_dirs: vec!["ray".into(), "ray/shared".into()],
		entries,
		linkages,
		create_bevy_shader: false,
	}
}

fn voxel_module_path(type_id: u16) -> PathBuf {
	PathBuf::from(format!("voxel/{type_id}.slang"))
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn compiled_stage_order_matches_pipeline_sources() {
		let stages = (0..12).map(|i| i.to_string()).collect::<Vec<_>>();
		let sources = VoxelShaderSources::from_compiled(&stages).unwrap();
		assert_eq!(sources.beam, "0");
		assert_eq!(sources.raycasting, "1");
		assert_eq!(sources.coloring, "2\n3");
		assert_eq!(sources.fallback, "4");
		for (i, source) in sources.face_gi.iter().enumerate() {
			assert_eq!(*source, (i + 5).to_string());
		}
		assert!(VoxelShaderSources::from_compiled(&stages[..4]).is_err());
		assert!(VoxelShaderSources::from_compiled(&stages[..11]).is_err());
	}
}
