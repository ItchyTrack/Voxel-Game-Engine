use std::{future::Future, sync::Arc};

use bevy::prelude::*;
use rustc_hash::FxHashMap;
use voxel_data::{
	grid::GridId,
	voxels::{VoxelTypeId, Voxels},
};

use crate::{NonZeroChunkRegion, TileClassId, TileData, TileGenerationParameters, TileKey, class::TileGenerationData};

pub use async_trait::async_trait;

#[async_trait]
pub trait TileGenerator: Send + Sync + 'static {
	async fn generate(&self, session: TileGenerationSession) -> Option<Box<dyn TileData>>;
}

#[derive(Resource, Default)]
pub struct TileGeneratorRegistry {
	generators: FxHashMap<TileClassId, Arc<dyn TileGenerator>>,
}

impl TileGeneratorRegistry {
	pub fn insert<G: TileGenerator>(&mut self, tile_class_id: TileClassId, generator: G) {
		self.generators.insert(tile_class_id, Arc::new(generator));
	}

	pub fn generator(&self, class: TileClassId) -> Arc<dyn TileGenerator> {
		self.generators.get(&key).cloned().unwrap_or_else(|| panic!("no tile generator registered for {key:?}"))
	}
}

pub trait TileAppExt {
	fn register_tile_generator<G: TileGenerator>(&mut self, tile_class_id: TileClassId, generator: G) -> &mut Self;
}

impl TileAppExt for App {
	fn register_tile_generator<G: TileGenerator>(&mut self, tile_class_id: TileClassId, generator: G) -> &mut Self {
		self.init_resource::<TileGeneratorRegistry>();
		self.world_mut().resource_mut::<TileGeneratorRegistry>().insert(tile_class_id, generator);
		self
	}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VoxelRegionRequest {
	pub area: NonZeroChunkRegion,
	pub lod: u8,
	pub voxel_type: VoxelTypeId,
}

#[derive(Debug)]
pub struct VoxelRegionResult {
	pub area: NonZeroChunkRegion,
	pub lod: u8,
	pub voxels: Voxels,
}

pub type ReceiveVoxelsFuture<'a> = std::pin::Pin<Box<dyn Future<Output = Option<VoxelRegionResult>> + Send + 'a>>;

pub trait GenerationVoxelReader: Send + 'static {
	fn request_voxels(&mut self, request: VoxelRegionRequest);
	fn receive_voxels(&mut self) -> ReceiveVoxelsFuture<'_>;
}

pub struct TileGenerationSession {
	pub grid: GridId,
	pub key: TileKey,
	chunk_size: u32,
	context: TileGenerationParameters,
	reader: Box<dyn GenerationVoxelReader>,
}

impl TileGenerationSession {
	pub fn new(
		grid: GridId,
		key: TileKey,
		chunk_size: u32,
		context: TileGenerationParameters,
		reader: Box<dyn GenerationVoxelReader>,
	) -> Self {
		Self { grid, key, chunk_size, context, reader }
	}

	pub fn context<T: TileGenerationData + 'static>(&self) -> &T {
		self.context.downcast_ref().unwrap_or_else(|| panic!("tile generator received generation context of the wrong type"))
	}

	pub fn request_voxels(&mut self, request: VoxelRegionRequest) {
		self.reader.request_voxels(request);
	}

	pub fn receive_voxels(&mut self) -> ReceiveVoxelsFuture<'_> {
		self.reader.receive_voxels()
	}

	pub async fn receive_merged_voxels(&mut self, area: NonZeroChunkRegion) -> Option<VoxelRegionResult> {
		let mut merged: Option<Voxels> = None;
		let mut lod = None;
		while let Some(result) = self.receive_voxels().await {
			let result_lod = *lod.get_or_insert(result.lod);
			assert_eq!(result.lod, result_lod, "cannot merge voxel-area results with different LODs");
			let step = 1i32.checked_shl(result.lod as u32).expect("voxel-area result LOD is too large");
			let offset = ((result.area.min() - area.min()) * self.chunk_size as i32).div_euclid(bevy::math::IVec3::splat(step));
			let target = merged.get_or_insert_with(|| Voxels::new_with_type(result.voxels.voxel_type_info()));
			assert_eq!(target.voxel_type_id(), result.voxels.voxel_type_id(), "cannot merge voxel-area results with different voxel types");
			target.merge_from(&result.voxels, offset);
		}
		merged.filter(|voxels| !voxels.is_empty()).map(|voxels| VoxelRegionResult {
			area,
			lod: lod.unwrap(),
			voxels,
		})
	}
}
