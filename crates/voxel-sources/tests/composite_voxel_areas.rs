use bevy::math::{IVec3, U16Vec3};
use bevy::prelude::Entity;
use voxel_data::voxels::{Voxel, VoxelTypeId, VoxelTypeInfo, Voxels};
use voxel_data::grid::GridId;
use voxel_sources::{CancellationToken, ChunkSource, SourceCoverage, SourceHandle, TakeJob, VoxelLodGenerator};

#[derive(Default)]
struct AreaSource {
	chunks: Vec<IVec3>,
}

impl ChunkSource for AreaSource {
	fn init(&self, _handle: SourceHandle) {}
	fn request_available_area(&self, _grid: GridId) {}
	fn request_load(&self, _grid: GridId, chunk: IVec3, _generation: u64, _cancellation: CancellationToken) -> SourceCoverage {
		if self.chunks.contains(&chunk) { SourceCoverage::All } else { SourceCoverage::None }
	}
	fn request_voxel_area(&self, _grid: GridId, min: IVec3, size: IVec3, _lod: f32, _voxel_type: VoxelTypeId, _generation: u64, _cancellation: CancellationToken) -> SourceCoverage {
		let owned = self.chunks.iter().filter(|chunk| chunk.cmpge(min).all() && chunk.cmplt(min + size).all()).count();
		SourceCoverage::from_count(owned, (size.x * size.y * size.z) as usize)
	}
	fn take(&self, _destination: voxel_sources::SourceId, _grid: GridId, _min: IVec3, _size: IVec3, _generation: u64) -> Vec<TakeJob> { Vec::new() }
	fn forget(&self, _grid: GridId, _chunk: IVec3) {}
}

fn test_type_info() -> VoxelTypeInfo {
	VoxelTypeInfo { id: VoxelTypeId(99), size_bytes: 8 }
}

fn voxel(color: [u8; 4], mass: u32) -> Voxel {
	let mut bytes = [0u8; 8];
	bytes[..4].copy_from_slice(&color);
	bytes[4..8].copy_from_slice(&mass.to_le_bytes());
	Voxel::new(test_type_info().id, bytes)
}

struct MarkerLodGenerator;

impl VoxelLodGenerator for MarkerLodGenerator {
	fn input_type_id(&self) -> VoxelTypeId { VoxelTypeId(99) }
	fn output_type_id(&self) -> VoxelTypeId { VoxelTypeId(99) }

	fn generate(&self, _min: IVec3, _size: IVec3, lod: f32, _fetch: &dyn Fn(IVec3) -> Option<Voxels>) -> Option<Voxels> {
		let mut voxels = Voxels::new_with_type(test_type_info());
		voxels.add_voxel(U16Vec3::new(lod as u16, 0, 0), voxel([255, 0, 0, 255], 0).get_ref());
		Some(voxels)
	}
}

#[test]
fn chunk_source_voxel_area_cost_detects_overlap() {
	let source = AreaSource { chunks: vec![IVec3::new(4, 0, 0)] };
	let grid = Entity::PLACEHOLDER;

	assert_eq!(source.request_voxel_area(grid, IVec3::ZERO, IVec3::new(8, 1, 1), 1.0, VoxelTypeId(99), 0, CancellationToken::new()), SourceCoverage::Some);
	assert_eq!(source.request_voxel_area(grid, IVec3::new(8, 0, 0), IVec3::new(4, 1, 1), 1.0, VoxelTypeId(99), 0, CancellationToken::new()), SourceCoverage::None);
}

#[test]
fn voxel_downsampler_contract_accepts_region_fetch_and_relative_lod() {
	let mut input = Voxels::new_with_type(test_type_info());
	input.add_voxel(U16Vec3::ZERO, voxel([0, 255, 0, 255], 1).get_ref());

	let generated = MarkerLodGenerator.generate(IVec3::ZERO, IVec3::ONE, 2.0, &|_| Some(input.clone())).expect("downsampler should produce marker voxels");

	assert!(generated.voxel(&U16Vec3::new(2, 0, 0)).is_some());
}
