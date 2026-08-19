use std::any::Any;

use basic_voxel::{BasicVoxel, MarchingVoxel};
use bevy::math::{IVec3, U16Vec3, UVec3, Vec3};
use tile_data::{NonZeroChunkRegion, TileData, TileGenerationSession, TileGenerator};
use voxel_data::voxels::{VoxelType, Voxels};
use tile_data::CHUNK_SIZE;

use voxel_gpu::packed_buffer_group::{PackedBufferGroupAllocation, PackedBufferGroupId};

use crate::{gpu_data::MarchingWorldGpuData, marching_cubes::make_marching_cubes_mesh_in_region};

#[derive(Debug)]
pub struct MarchingTileData {
	pub vertices: PackedBufferGroupAllocation,
	pub vertex_count: u32,
	pub bounds_min: Vec3,
	pub bounds_max: Vec3,
	pub voxel_lod: u8,
}

impl TileData for MarchingTileData {
	fn as_any(&self) -> &dyn Any { self }
}

#[derive(Clone, Copy, Debug)]
pub struct MarchingTileCapabilityData {
	pub vertices: PackedBufferGroupId,
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
			vertices: self.vertices.id(),
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
		let area = session.key.region;
		let padded_area = NonZeroChunkRegion::new(area.min() - IVec3::ONE, area.size() + UVec3::splat(2)).unwrap();
		session.request_voxels(
			padded_area,
			session.key.lod,
			MarchingVoxel::TYPE_INFO.id,
		);
		let input = session.receive_merged_voxels_with(padded_area, into_marching_voxels).await?;
		let step = 1i32.checked_shl(input.lod as u32)?;
		let cell_min = IVec3::splat(CHUNK_SIZE.div_euclid(step));
		let unscaled_size = (area.size() * CHUNK_SIZE as u32).as_ivec3();
		let cell_size = (unscaled_size + IVec3::splat(step - 1)).div_euclid(IVec3::splat(step));
		let (vertices, vertex_count, bounds_min, bounds_max) =
			make_marching_cubes_mesh_in_region(&input.voxels, cell_min, cell_size);
		if vertex_count == 0 { return None; }
		let vertices = self.gpu.create_vertex_buffer(bytemuck::cast_slice(&vertices).to_vec());
		Some(Box::new(MarchingTileData {
			vertices,
			vertex_count,
			bounds_min,
			bounds_max,
			voxel_lod: input.lod,
		}))
	}
}

fn into_marching_voxels(voxels: Voxels) -> Voxels {
	match voxels.voxel_type_id() {
		MarchingVoxel::TYPE_ID => voxels,
		BasicVoxel::TYPE_ID => {
			let mut converted = Voxels::new::<MarchingVoxel>();
			for (position, size, voxel) in voxels.grid_tree().iter() {
				let voxel = MarchingVoxel(BasicVoxel::from_voxel_ref(&voxel));
				converted.add_area(position, U16Vec3::splat(size), voxel.get_ref());
			}
			converted
		}
		type_id => panic!("marching tile generator cannot convert voxel type {type_id:?} to MarchingVoxel"),
	}
}
