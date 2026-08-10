use std::any::Any;

use basic_voxel::MarchingVoxel;
use bevy::math::{IVec3, Vec3};
use tile_data::{TileData, TileGenerationSession, TileGenerator, VoxelArea, VoxelAreaRequest};
use voxel_data::voxels::VoxelType;
use voxel_streaming::CHUNK_SIZE;

use crate::{
	gpu_data::{MarchingGpuBuffer, MarchingWorldGpuData},
	marching_cubes::make_marching_cubes_mesh_in_region,
};

#[derive(Debug)]
pub struct MarchingTileData {
	pub vertices: MarchingGpuBuffer,
	pub vertex_count: u32,
	pub bounds_min: Vec3,
	pub bounds_max: Vec3,
	pub voxel_lod: u8,
}

impl TileData for MarchingTileData {
	fn as_any(&self) -> &dyn Any { self }
}

#[derive(Clone, Debug)]
pub struct MarchingTileCapabilityData {
	pub vertices: MarchingGpuBuffer,
	pub vertex_count: u32,
	pub bounds_min: Vec3,
	pub bounds_max: Vec3,
	pub voxel_lod: u8,
}

pub trait MarchingTileCapability: TileData {
	fn marching_tile_data(&self) -> MarchingTileCapabilityData;
}

impl MarchingTileCapability for MarchingTileData {
	fn marching_tile_data(&self) -> MarchingTileCapabilityData {
		MarchingTileCapabilityData {
			vertices: self.vertices.clone(),
			vertex_count: self.vertex_count,
			bounds_min: self.bounds_min,
			bounds_max: self.bounds_max,
			voxel_lod: self.voxel_lod,
		}
	}
}

pub struct MarchingTileGenerator {
	pub gpu: MarchingWorldGpuData,
}

#[tile_data::async_trait]
impl TileGenerator for MarchingTileGenerator {
	async fn generate(&self, mut session: TileGenerationSession) -> Option<Box<dyn TileData>> {
		let area = VoxelArea { min: session.key.min, size: session.key.size };
		let padded_area = VoxelArea {
			min: area.min - IVec3::ONE,
			size: area.size + IVec3::splat(2),
		};
		session.request_voxels(VoxelAreaRequest {
			area: padded_area,
			lod: session.key.lod,
			voxel_type: MarchingVoxel::TYPE_INFO.id,
		});
		let input = session.receive_merged_voxels(padded_area).await?;
		let step = 1i32.checked_shl(input.lod as u32)?;
		let cell_min = IVec3::splat(CHUNK_SIZE.div_euclid(step));
		let unscaled_size = area.size * CHUNK_SIZE;
		let cell_size = (unscaled_size + IVec3::splat(step - 1)).div_euclid(IVec3::splat(step));
		let (vertices, vertex_count, bounds_min, bounds_max) =
			make_marching_cubes_mesh_in_region(&input.voxels, cell_min, cell_size);
		if vertex_count == 0 { return None; }
		let vertices = self.gpu.create_vertex_buffer(bytemuck::cast_slice(&vertices));
		Some(Box::new(MarchingTileData {
			vertices,
			vertex_count,
			bounds_min,
			bounds_max,
			voxel_lod: input.lod,
		}))
	}
}
