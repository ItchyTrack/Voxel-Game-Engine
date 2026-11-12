use bevy::{math::UVec3, prelude::Component};
use bile_tiles::TileKey;
use tile_data::{TileVoxelReducerRegistry, VoxelRegionResult};
use voxel_data::grid::GridId;
use voxel_sources::VoxelRequest;
use voxel_streaming::VoxelTileBuilder;
use voxel_data::voxels::VoxelTypeId;
use voxel_gpu::{VoxelGpuDataReaders, packed_buffer_group::PackedBufferGroupAllocation};

use crate::{gpu_data::RasterWorldGpuData, gpu_raster_mesh::make_gpu_raster_mesh};

#[derive(Component, Default)]
pub struct RasterTileClass;

#[derive(Component, Debug, Clone)]
pub struct RasterTileData {
	pub faces: PackedBufferGroupAllocation,
	pub voxel_data: PackedBufferGroupAllocation,
	pub face_count: u32,
	pub bounds_min: UVec3,
	pub bounds_max: UVec3,
	pub voxel_type: VoxelTypeId,
	pub voxel_lod: u8,
}

pub struct VoxelRasterTileBuilder {
	pub voxel_type: VoxelTypeId,
	pub lod_levels: u8,
	pub gpu: RasterWorldGpuData,
	pub readers: VoxelGpuDataReaders,
	pub reducers: TileVoxelReducerRegistry,
}

impl VoxelTileBuilder for VoxelRasterTileBuilder {
	type Output = RasterTileData;

	fn voxels(&self, grid: GridId, key: TileKey) -> VoxelRequest {
		VoxelRequest { grid, area: key.region, lod: key.lod.saturating_sub(self.lod_levels), voxel_type: Some(self.voxel_type) }
	}

	fn build(&self, _grid: GridId, key: TileKey, inputs: &[VoxelRegionResult]) -> Option<RasterTileData> {
		let area = key.region;
		let voxel_lod = key.lod.saturating_sub(self.lod_levels);
		let voxels = self.reducers.reduce(area, voxel_lod, self.voxel_type, inputs)?;
		let source_voxel_type = voxels.voxel_type_id();
		assert!(
			self.readers.contains(source_voxel_type),
			"raster tile builder cannot generate voxel type {source_voxel_type:?}",
		);
		let (bounds_min, bounds_max) = voxels.bounding_box()?;
		let voxel_type = voxels.voxel_type_info();
		let (faces, voxel_data, face_count) = make_gpu_raster_mesh(voxels.grid_tree(), voxel_type, &self.readers);
		if face_count == 0 { return None; }
		let mut gpu = self.gpu.lock();
		let faces = gpu.faces.add_buffer(faces).expect("failed to allocate raster tile face data");
		let voxel_data = gpu.voxel_data.add_buffer(voxel_data).expect("failed to allocate raster tile voxel data");
		Some(RasterTileData {
			faces,
			voxel_data,
			face_count,
			bounds_min,
			bounds_max,
			voxel_type: voxel_type.id,
			voxel_lod,
		})
	}
}
