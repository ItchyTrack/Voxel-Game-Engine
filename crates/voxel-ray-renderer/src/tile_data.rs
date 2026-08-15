use std::any::Any;

use bevy::math::U16Vec3;
use tile_data::{TileData, TileGenerationSession, TileGenerator, VoxelAreaRequest};
use voxel_data::voxels::{VoxelTypeId, VoxelTypeInfo};
use voxel_gpu::{AllocationId, PackedBufferAllocation, VoxelGpuDataReaders};

use crate::{gpu_data::RayWorldGpuData, gpu_grid_tree::make_gpu_grid_tree};

#[derive(Clone, Copy, Debug)]
pub struct RayTilePlacement {
	pub tree_root_pos: U16Vec3,
	pub bounds_min: U16Vec3,
	pub bounds_max: U16Vec3,
}

#[derive(Debug)]
pub struct RayTileData {
	pub tree: PackedBufferAllocation,
	pub voxels: PackedBufferAllocation,
	pub generation: u64,
	pub placement: RayTilePlacement,
	pub voxel_type: VoxelTypeInfo,
	pub voxel_lod: u8,
}

impl TileData for RayTileData {
	fn as_any(&self) -> &dyn Any { self }
}

#[derive(Clone, Copy, Debug)]
pub struct RayTileCapabilityData {
	pub tree: AllocationId,
	pub voxels: AllocationId,
	pub generation: u64,
	pub placement: RayTilePlacement,
	pub voxel_type: VoxelTypeInfo,
	pub voxel_lod: u8,
}

/// Supplies the normalized data consumed by the ray renderer.
pub trait RayTileCapability: TileData {
	fn ray_tile_data(&self) -> RayTileCapabilityData;
}

impl RayTileCapability for RayTileData {
	fn ray_tile_data(&self) -> RayTileCapabilityData {
		RayTileCapabilityData {
			tree: self.tree.id(),
			voxels: self.voxels.id(),
			generation: self.generation,
			placement: self.placement,
			voxel_type: self.voxel_type,
			voxel_lod: self.voxel_lod,
		}
	}
}

pub struct VoxelRayTileGenerator {
	pub voxel_type: VoxelTypeId,
	pub lod_levels: u8,
	pub gpu: RayWorldGpuData,
	pub readers: VoxelGpuDataReaders,
}

#[tile_data::async_trait]
impl TileGenerator for VoxelRayTileGenerator {
	async fn generate(&self, mut session: TileGenerationSession) -> Option<Box<dyn TileData>> {
		let region = session.key.region;
		session.request_voxels(VoxelAreaRequest {
			area: region,
			lod: session.key.lod.saturating_sub(self.lod_levels),
			voxel_type: self.voxel_type,
		});
		let input = session.receive_merged_voxels(region).await?;
		let (bounds_min, bounds_max) = input.voxels.bounding_box()?;
		let placement = RayTilePlacement {
			tree_root_pos: input.voxels.grid_tree().view().root_pos(),
			bounds_min,
			bounds_max,
		};
		let voxel_type = input.voxels.voxel_type_info();
		let (tree, voxels) = make_gpu_grid_tree(input.voxels.grid_tree(), voxel_type, &self.readers);
		let mut gpu = self.gpu.lock();
		Some(Box::new(RayTileData {
			tree: gpu.trees.add_buffer(tree).expect("failed to allocate ray tile tree data"),
			voxels: gpu.voxels.add_buffer(voxels).expect("failed to allocate ray tile voxel data"),
			generation: gpu.next_generation(),
			placement,
			voxel_type,
			voxel_lod: input.lod,
		}))
	}
}
