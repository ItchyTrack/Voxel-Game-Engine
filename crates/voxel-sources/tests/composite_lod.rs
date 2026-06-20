use bevy::math::{I16Vec3, IVec3};
use voxel_data::voxels::{Voxel, Voxels};
use voxel_data::grid::GridId;
use voxel_sources::{ChunkSource, SourceHandle, VoxelLodGenerator};

#[derive(Default)]
struct AreaSource {
	chunks: Vec<IVec3>,
}

impl ChunkSource for AreaSource {
	fn init(&self, _handle: SourceHandle) {}
	fn cost(&self, _grid: GridId, chunk: IVec3) -> Option<u32> { self.chunks.contains(&chunk).then_some(0) }
	fn request_load(&self, _grid: GridId, _chunk: IVec3) {}
	fn cost_lod(&self, _grid: GridId, min: IVec3, size: IVec3, _lod: f32) -> Option<u32> {
		self.chunks
			.iter()
			.any(|chunk| chunk.cmpge(min).all() && chunk.cmplt(min + size).all())
			.then_some(0)
	}
	fn request_load_lod(&self, _grid: GridId, _min: IVec3, _size: IVec3, _lod: f32) {}
}

struct MarkerGenerator;

impl VoxelLodGenerator for MarkerGenerator {
	fn generate(&self, _voxels: &Voxels, lod: f32) -> Option<Voxels> {
		let mut voxels = Voxels::new();
		voxels.add_voxel(I16Vec3::new(lod as i16, 0, 0), Voxel { color: [255, 0, 0, 255], mass: 0 });
		Some(voxels)
	}
}

#[test]
fn chunk_source_lod_cost_detects_overlap() {
	let source = AreaSource { chunks: vec![IVec3::new(4, 0, 0)] };
	let grid = Entity::PLACEHOLDER;

	assert_eq!(source.cost_lod(grid, IVec3::ZERO, IVec3::new(8, 1, 1), 1.0), Some(0));
	assert_eq!(source.cost_lod(grid, IVec3::new(8, 0, 0), IVec3::new(4, 1, 1), 1.0), None);
}

#[test]
fn voxel_lod_generator_contract_accepts_voxels_and_relative_lod() {
	let mut input = Voxels::new();
	input.add_voxel(I16Vec3::ZERO, Voxel { color: [0, 255, 0, 255], mass: 1 });

	let generated = MarkerGenerator.generate(&input, 2.0).expect("generator should produce marker voxels");

	assert!(generated.voxel(&I16Vec3::new(2, 0, 0)).is_some());
}
