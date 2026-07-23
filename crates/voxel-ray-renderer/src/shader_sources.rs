use std::path::Path;

use voxel_gpu::shader_compiler::{ShaderResult, SlangConformance, SlangEntry, SlangLinkage, SlangModule, SlangStage};
use voxel_gpu::{VoxelGpuDataReaders, VoxelShaderRegistration};

const VOXEL_SHADER_INTERFACE: &str = "IVoxelSampler";

const ENTRIES: &[SlangEntry<'_>] = &[
	SlangEntry { source: "beam.slang", entry: "main", stage: SlangStage::Compute },
	SlangEntry { source: "raycasting.slang", entry: "main", stage: SlangStage::Compute },
	SlangEntry { source: "coloring_shader.slang", entry: "vs_main", stage: SlangStage::Vertex },
	SlangEntry { source: "coloring_shader.slang", entry: "fs_main", stage: SlangStage::Fragment },
];

#[derive(Clone)]
pub struct VoxelShaderSources {
	pub beam: String,
	pub raycasting: String,
	pub coloring: String,
}

impl VoxelShaderSources {
	pub fn embedded(voxel_data_readers: &VoxelGpuDataReaders) -> ShaderResult<Self> {
		let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
		let local_shader_dir = manifest_dir.join("src/shaders");
		let shared_shader_dir = voxel_gpu::shader_sources::shared_shader_dir_from_manifest(manifest_dir);
		Self::from_compiled(voxel_gpu::shader_sources::compile_embedded(&local_shader_dir, &shared_shader_dir, ENTRIES, &linkages(voxel_data_readers))?)
	}

	#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
	pub fn load_from_disk(voxel_data_readers: &VoxelGpuDataReaders) -> ShaderResult<Self> {
		let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
		let local_shader_dir = manifest_dir.join("src/shaders");
		let shared_shader_dir = voxel_gpu::shader_sources::shared_shader_dir_from_manifest(manifest_dir);
		Self::from_compiled(voxel_gpu::shader_sources::compile_from_disk(&local_shader_dir, &shared_shader_dir, ENTRIES, &linkages(voxel_data_readers))?)
	}

	fn from_compiled(mut shaders: Vec<String>) -> ShaderResult<Self> {
		if shaders.len() != 4 {
			return Err(std::io::Error::other("expected four compiled voxel ray shader stages").into());
		}
		let fs = shaders.pop().unwrap();
		let vs = shaders.pop().unwrap();
		let raycasting = shaders.pop().unwrap();
		let beam = shaders.pop().unwrap();
		Ok(Self { beam, raycasting, coloring: format!("{vs}\n{fs}") })
	}
}

pub fn shader_source_signature(voxel_data_readers: &VoxelGpuDataReaders) -> Vec<VoxelShaderRegistration> {
	voxel_data_readers.shader_sources()
}

fn linkages(voxel_data_readers: &VoxelGpuDataReaders) -> Vec<SlangLinkage> {
	let voxel_linkage = voxel_linkage(voxel_data_readers);
	vec![
		SlangLinkage::default(),
		SlangLinkage::default(),
		voxel_linkage.clone(),
		voxel_linkage,
	]
}

fn voxel_linkage(voxel_data_readers: &VoxelGpuDataReaders) -> SlangLinkage {
	let shader_sources = voxel_data_readers.shader_sources();
	SlangLinkage {
		modules: shader_sources.iter().map(|(_, source, _)| SlangModule { path: (*source).into() }).collect(),
		conformances: shader_sources.iter()
			.map(|(type_id, _, sampler)| SlangConformance {
				ty: (*sampler).to_string(),
				interface: VOXEL_SHADER_INTERFACE.to_string(),
				id: type_id.0,
			})
			.collect(),
	}
}
