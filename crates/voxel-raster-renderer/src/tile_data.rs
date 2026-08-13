use std::any::Any;

use bevy::math::U16Vec3;
use tile_data::{TileData, TileGenerationSession, TileGenerator, ChunkRegion, VoxelAreaRequest};
use voxel_data::voxels::VoxelTypeId;
use voxel_gpu::{VoxelGpuDataReaders, packed_buffer_group::{PackedBufferGroupAllocation, PackedBufferGroupId}};

use crate::{gpu_data::RasterWorldGpuData, gpu_raster_mesh::make_gpu_raster_mesh};

#[derive(Debug)]
pub struct RasterTileData {
	pub faces: PackedBufferGroupAllocation,
	pub palette: PackedBufferGroupAllocation,
	pub face_count: u32,
	pub bounds_min: U16Vec3,
	pub bounds_max: U16Vec3,
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
	pub bounds_min: U16Vec3,
	pub bounds_max: U16Vec3,
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

pub struct VoxelRasterTileGenerator {
	pub voxel_type: VoxelTypeId,
	pub lod_levels: u8,
	pub gpu: RasterWorldGpuData,
	pub readers: VoxelGpuDataReaders,
}

#[tile_data::async_trait]
impl TileGenerator for VoxelRasterTileGenerator {
	async fn generate(&self, mut session: TileGenerationSession) -> Option<Box<dyn TileData>> {
		let area = ChunkRegion::new(session.key.min(), session.key.size());
		session.request_voxels(VoxelAreaRequest {
			area,
			lod: session.key.lod.saturating_sub(self.lod_levels),
			voxel_type: self.voxel_type,
		});
		let input = session.receive_merged_voxels(area).await?;
		let (bounds_min, bounds_max) = input.voxels.bounding_box()?;
		assert_eq!(input.voxels.voxel_type_id(), self.voxel_type);
		let voxel_type = input.voxels.voxel_type_info();
		let (faces, palette, face_count) = make_gpu_raster_mesh(input.voxels.grid_tree(), voxel_type, &self.readers);
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
			voxel_lod: input.lod,
		}))
	}
}
