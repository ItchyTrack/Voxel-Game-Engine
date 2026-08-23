use std::any::Any;

use bevy::math::UVec3;
use tile_data::{TileData, TileBuildingSession, TileBuilder, TileVoxelReducerRegistry};
use voxel_data::voxels::VoxelTypeId;
use voxel_gpu::{VoxelGpuDataReaders, packed_buffer_group::{PackedBufferGroupAllocation, PackedBufferGroupId}};

use crate::{gpu_data::RasterWorldGpuData, gpu_raster_mesh::make_gpu_raster_mesh};

#[derive(Debug)]
pub struct RasterTileData {
	pub faces: PackedBufferGroupAllocation,
	pub palette: PackedBufferGroupAllocation,
	pub face_count: u32,
	pub bounds_min: UVec3,
	pub bounds_max: UVec3,
	pub voxel_lod: u8,
}

impl TileData for RasterTileData {
	fn as_any(&self) -> &dyn Any { self }
}

#[derive(Clone, Copy, Debug)]
pub struct RasterTileCapabilityData {
	pub faces: PackedBufferGroupId,
	pub palette: PackedBufferGroupId,
	pub face_count: u32,
	pub bounds_min: UVec3,
	pub bounds_max: UVec3,
	pub voxel_lod: u8,
}

/// Supplies the normalized data consumed by the raster renderer.
pub trait RasterTileCapability: TileData {
	fn raster_tile_data(&self) -> RasterTileCapabilityData;
}

impl RasterTileCapability for RasterTileData {
	fn raster_tile_data(&self) -> RasterTileCapabilityData {
		RasterTileCapabilityData {
			faces: self.faces.id(),
			palette: self.palette.id(),
			face_count: self.face_count,
			bounds_min: self.bounds_min,
			bounds_max: self.bounds_max,
			voxel_lod: self.voxel_lod,
		}
	}
}

pub struct VoxelRasterTileBuilder {
	pub voxel_type: VoxelTypeId,
	pub lod_levels: u8,
	pub gpu: RasterWorldGpuData,
	pub readers: VoxelGpuDataReaders,
	pub reducers: TileVoxelReducerRegistry,
}

#[tile_data::async_trait]
impl TileBuilder for VoxelRasterTileBuilder {
	async fn build(&self, mut session: TileBuildingSession) -> Option<Box<dyn TileData>> {
		let area = session.key.region;
		let voxel_lod = session.key.lod.saturating_sub(self.lod_levels);
		session.request_voxels(area, voxel_lod, self.voxel_type);
		let mut inputs = Vec::new();
		while let Some(input) = session.receive_voxels().await {
			inputs.push(input);
		}
		let voxels = self.reducers.reduce(area, voxel_lod, self.voxel_type, &inputs)?;
		let source_voxel_type = voxels.voxel_type_id();
		assert!(
			self.readers.contains(source_voxel_type),
			"raster tile builder cannot generate voxel type {source_voxel_type:?}",
		);
		let (bounds_min, bounds_max) = voxels.bounding_box()?;
		let voxel_type = voxels.voxel_type_info();
		let (faces, palette, face_count) = make_gpu_raster_mesh(voxels.grid_tree(), voxel_type, &self.readers);
		if face_count == 0 { return None; }
		let mut gpu = self.gpu.lock();
		let faces = gpu.faces.add_buffer(faces).expect("failed to allocate raster tile face data");
		let palette = gpu.palettes.add_buffer(palette).expect("failed to allocate raster tile palette data");
		Some(Box::new(RasterTileData {
			faces,
			palette,
			face_count,
			bounds_min,
			bounds_max,
			voxel_lod,
		}))
	}
}
