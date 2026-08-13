use std::{future::Future, sync::Arc};

use voxel_data::{
	grid::GridId,
	voxels::{VoxelTypeId, Voxels},
};

use crate::{ChunkRegion, TileData, TileGenerationContext, TileKey};

pub use async_trait::async_trait;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VoxelAreaRequest {
	pub area: ChunkRegion,
	pub lod: u8,
	pub voxel_type: VoxelTypeId,
}

#[derive(Debug)]
pub struct VoxelAreaResult {
	pub area: ChunkRegion,
	pub lod: u8,
	pub voxels: Voxels,
}

pub type ReceiveVoxelsFuture<'a> = std::pin::Pin<Box<dyn Future<Output = Option<VoxelAreaResult>> + Send + 'a>>;

pub trait GenerationVoxelReader: Send + 'static {
	fn request_voxels(&mut self, request: VoxelAreaRequest);
	fn receive_voxels(&mut self) -> ReceiveVoxelsFuture<'_>;
}

pub struct TileGenerationSession {
	pub grid: GridId,
	pub key: TileKey,
	chunk_size: i32,
	context: TileGenerationContext,
	reader: Box<dyn GenerationVoxelReader>,
}

impl TileGenerationSession {
	pub fn new(
		grid: GridId,
		key: TileKey,
		chunk_size: i32,
		context: TileGenerationContext,
		reader: Box<dyn GenerationVoxelReader>,
	) -> Self {
		Self { grid, key, chunk_size, context, reader }
	}

	pub fn context<T: Send + Sync + 'static>(&self) -> &T {
		self.context.downcast_ref().unwrap_or_else(|| panic!("tile generator received generation context of the wrong type"))
	}

	pub fn request_voxels(&mut self, request: VoxelAreaRequest) {
		self.reader.request_voxels(request);
	}

	pub fn receive_voxels(&mut self) -> ReceiveVoxelsFuture<'_> {
		self.reader.receive_voxels()
	}

	pub async fn receive_merged_voxels(&mut self, area: ChunkRegion) -> Option<VoxelAreaResult> {
		let mut merged: Option<Voxels> = None;
		let mut lod = None;
		while let Some(result) = self.receive_voxels().await {
			let result_lod = *lod.get_or_insert(result.lod);
			assert_eq!(result.lod, result_lod, "cannot merge voxel-area results with different LODs");
			let step = 1i32.checked_shl(result.lod as u32).expect("voxel-area result LOD is too large");
			let offset = ((result.area.min() - area.min()) * self.chunk_size).div_euclid(bevy::math::IVec3::splat(step));
			let target = merged.get_or_insert_with(|| Voxels::new_with_type(result.voxels.voxel_type_info()));
			assert_eq!(target.voxel_type_id(), result.voxels.voxel_type_id(), "cannot merge voxel-area results with different voxel types");
			target.merge_from(&result.voxels, offset);
		}
		merged.filter(|voxels| !voxels.is_empty()).map(|voxels| VoxelAreaResult {
			area,
			lod: lod.unwrap(),
			voxels,
		})
	}
}

#[async_trait]
pub trait TileGenerator: Send + Sync + 'static {
	async fn generate(&self, session: TileGenerationSession) -> Option<Box<dyn TileData>>;
}

pub type SharedTileGenerator = Arc<dyn TileGenerator>;
