use basic_voxel::MarchingVoxel;
use bevy::{math::{IVec3, UVec3, Vec3}, prelude::Component};
use bile_math::NonZeroRegion;
use bile_tiles::TileKey;
use tile_data::{TileVoxelReducerRegistry, VoxelRegionResult};
use voxel_data::grid::GridId;
use voxel_sources::VoxelRequest;
use voxel_streaming::VoxelTileBuilder;
use voxel_data::voxels::VoxelType;
use tile_data::CHUNK_SIZE;

use voxel_gpu::packed_buffer_group::PackedBufferGroupAllocation;

use crate::{gpu_data::MarchingWorldGpuData, marching_cubes::make_marching_cubes_mesh_in_region};

#[derive(Component, Default)]
pub struct MarchingTileClass;

#[derive(Component, Debug, Clone)]
pub struct MarchingTileData {
	pub vertices: PackedBufferGroupAllocation,
	pub vertex_count: u32,
	pub bounds_min: Vec3,
	pub bounds_max: Vec3,
	pub voxel_lod: u8,
}

pub struct MarchingTileBuilder {
	pub gpu: MarchingWorldGpuData,
	pub reducers: TileVoxelReducerRegistry,
}

fn padded_area(area: NonZeroRegion) -> NonZeroRegion {
	NonZeroRegion::new(area.min() - IVec3::splat(CHUNK_SIZE as i32), area.size() + UVec3::splat(2 * CHUNK_SIZE)).unwrap()
}

impl VoxelTileBuilder for MarchingTileBuilder {
	type Output = MarchingTileData;

	fn voxels(&self, grid: GridId, key: TileKey) -> VoxelRequest {
		VoxelRequest { grid, area: padded_area(key.region), lod: key.lod, voxel_type: Some(MarchingVoxel::TYPE_INFO.id) }
	}

	fn build(&self, _grid: GridId, key: TileKey, inputs: &[VoxelRegionResult]) -> Option<MarchingTileData> {
		let area = key.region;
		let padded_area = padded_area(area);
		let voxel_lod = key.lod;
		let voxels = self.reducers.reduce(padded_area, voxel_lod, MarchingVoxel::TYPE_INFO.id, inputs)?;
		let step = 1i32.checked_shl(voxel_lod as u32)?;
		let cell_min = IVec3::splat((CHUNK_SIZE as i32).div_euclid(step));
		let unscaled_size = area.size().as_ivec3();
		let cell_size = (unscaled_size + IVec3::splat(step - 1)).div_euclid(IVec3::splat(step));
		let (vertices, vertex_count, bounds_min, bounds_max) =
			make_marching_cubes_mesh_in_region(&voxels, cell_min, cell_size);
		if vertex_count == 0 { return None; }
		let vertices = self.gpu.create_vertex_buffer(bytemuck::cast_slice(&vertices).to_vec());
		Some(MarchingTileData {
			vertices,
			vertex_count,
			bounds_min,
			bounds_max,
			voxel_lod,
		})
	}
}
