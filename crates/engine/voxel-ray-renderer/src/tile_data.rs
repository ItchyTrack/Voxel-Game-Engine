use bevy::{math::UVec3, prelude::Component};
use bile_tiles::TileKey;
use tile_data::{TileVoxelReducerRegistry, VoxelRegionResult};
use voxel_data::grid::GridId;
use voxel_sources::VoxelRequest;
use voxel_streaming::VoxelTileBuilder;
use voxel_data::voxels::{VoxelTypeId, VoxelTypeInfo};
use voxel_gpu::{PackedBufferAllocation, VoxelGpuDataReaders};
use voxel_trees::views::GridTreeView;

use crate::{gpu_data::RayWorldGpuData, gpu_grid_tree::make_gpu_grid_tree};

#[derive(Clone, Copy, Debug)]
pub struct RayTilePlacement {
	pub tree_root_pos: UVec3,
	pub bounds_min: UVec3,
	pub bounds_max: UVec3,
}

#[derive(Component, Default)]
pub struct RayTileClass;

#[derive(Component, Debug, Clone)]
pub struct RayTileData {
	pub tree: PackedBufferAllocation,
	pub voxels: PackedBufferAllocation,
	pub generation: u64,
	pub placement: RayTilePlacement,
	pub voxel_type: VoxelTypeInfo,
	pub voxel_lod: u8,
}

pub struct VoxelRayTileBuilder {
	pub voxel_type: VoxelTypeId,
	pub lod_levels: u8,
	pub gpu: RayWorldGpuData,
	pub readers: VoxelGpuDataReaders,
	pub reducers: TileVoxelReducerRegistry,
}

impl VoxelTileBuilder for VoxelRayTileBuilder {
	type Output = RayTileData;

	fn voxels(&self, grid: GridId, key: TileKey) -> VoxelRequest {
		VoxelRequest { grid, area: key.region, lod: key.lod.saturating_sub(self.lod_levels), voxel_type: Some(self.voxel_type) }
	}

	fn build(&self, _grid: GridId, key: TileKey, inputs: &[VoxelRegionResult]) -> Option<RayTileData> {
		let region = key.region;
		let voxel_lod = key.lod.saturating_sub(self.lod_levels);
		let voxels = self.reducers.reduce(region, voxel_lod, self.voxel_type, inputs)?;
		let source_voxel_type = voxels.voxel_type_id();
		assert!(
			self.readers.contains(source_voxel_type),
			"ray tile builder cannot generate voxel type {source_voxel_type:?}",
		);
		let (bounds_min, bounds_max) = voxels.bounding_box()?;
		let placement = RayTilePlacement {
			tree_root_pos: voxels.grid_tree().root_pos(),
			bounds_min,
			bounds_max,
		};
		let voxel_type = voxels.voxel_type_info();
		let (tree, voxels) = make_gpu_grid_tree(voxels.grid_tree(), voxel_type, &self.readers);
		let mut gpu = self.gpu.lock();
		Some(RayTileData {
			tree: gpu.trees.add_buffer(tree).expect("failed to allocate ray tile tree data"),
			voxels: gpu.voxels.add_buffer(voxels).expect("failed to allocate ray tile voxel data"),
			generation: gpu.next_generation(),
			placement,
			voxel_type,
			voxel_lod,
		})
	}
}
