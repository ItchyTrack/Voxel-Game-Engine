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
		self.generators.get(&class).cloned().unwrap_or_else(|| panic!("no tile generator registered for {class:?}"))
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
	context: TileGenerationParameters,
	reader: Box<dyn GenerationVoxelReader>,
}

impl TileGenerationSession {
	pub fn new(
		grid: GridId,
		key: TileKey,
		context: TileGenerationParameters,
		reader: Box<dyn GenerationVoxelReader>,
	) -> Self {
		Self { grid, key, context, reader }
	}

	pub fn context<T: TileGenerationData + 'static>(&self) -> &T {
		self.context.downcast_ref().unwrap_or_else(|| panic!("tile generator received generation context of the wrong type"))
	}

	pub fn request_voxels(
		&mut self,
		area: NonZeroChunkRegion,
		lod: u8,
		voxel_type: VoxelTypeId,
	) {
		self.reader.request_voxels(VoxelRegionRequest {
			area,
			lod,
			voxel_type,
		});
	}

	pub fn receive_voxels(&mut self) -> ReceiveVoxelsFuture<'_> {
		self.reader.receive_voxels()
	}
}
