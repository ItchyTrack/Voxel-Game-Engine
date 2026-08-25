use std::any::Any;

use bevy::math::UVec3;
use tile_data::{TileData, TileBuildingSession, TileBuilder, TileVoxelReducerRegistry};
use voxel_data::voxels::{VoxelTypeId, VoxelTypeInfo};
use voxel_gpu::{PackedBufferAllocation, VoxelGpuDataReaders};

use crate::{gpu_data::RayWorldGpuData, gpu_grid_tree::make_gpu_grid_tree};

#[derive(Clone, Copy, Debug)]
pub struct RayTilePlacement {
	pub tree_root_pos: UVec3,
	pub bounds_min: UVec3,
	pub bounds_max: UVec3,
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

#[derive(Clone, Debug)]
pub struct RayTileCapabilityData {
	pub tree: PackedBufferAllocation,
	pub voxels: PackedBufferAllocation,
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
			tree: self.tree.clone(),
			voxels: self.voxels.clone(),
			generation: self.generation,
			placement: self.placement,
			voxel_type: self.voxel_type,
			voxel_lod: self.voxel_lod,
		}
	}
}

pub struct VoxelRayTileBuilder {
	pub voxel_type: VoxelTypeId,
	pub lod_levels: u8,
	pub gpu: RayWorldGpuData,
	pub readers: VoxelGpuDataReaders,
	pub reducers: TileVoxelReducerRegistry,
}

#[tile_data::async_trait]
impl TileBuilder for VoxelRayTileBuilder {
	async fn build(&self, mut session: TileBuildingSession) -> Option<Box<dyn TileData>> {
		let region = session.key.region;
		let voxel_lod = session.key.lod.saturating_sub(self.lod_levels);
		session.request_voxels(region, voxel_lod, Some(self.voxel_type));
		let mut inputs = Vec::new();
		while let Some(input) = session.receive_voxels().await {
			inputs.push(input);
		}
		let voxels = self.reducers.reduce(region, voxel_lod, self.voxel_type, &inputs)?;
		let source_voxel_type = voxels.voxel_type_id();
		assert!(
			self.readers.contains(source_voxel_type),
			"ray tile builder cannot generate voxel type {source_voxel_type:?}",
		);
		let (bounds_min, bounds_max) = voxels.bounding_box()?;
		let placement = RayTilePlacement {
			tree_root_pos: voxels.grid_tree().view().root_pos(),
			bounds_min,
			bounds_max,
		};
		let voxel_type = voxels.voxel_type_info();
		let (tree, voxels) = make_gpu_grid_tree(voxels.grid_tree(), voxel_type, &self.readers);
		let mut gpu = self.gpu.lock();
		Some(Box::new(RayTileData {
			tree: gpu.trees.add_buffer(tree).expect("failed to allocate ray tile tree data"),
			voxels: gpu.voxels.add_buffer(voxels).expect("failed to allocate ray tile voxel data"),
			generation: gpu.next_generation(),
			placement,
			voxel_type,
			voxel_lod,
		}))
	}
}
