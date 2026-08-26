use std::sync::Arc;

use bevy::{ecs::system::SystemParam, math::IVec3, prelude::*};
use tile_data::{NonZeroChunkRegion, chunks_covering_nonzero_voxel_region};
use voxel_data::grid::{Grid, GridId};
use voxel_sources::{
	SourceManager,
	edit::{GridEdit, GridEditId, GridEditIdManager, GridEditMessage, GridGeneration},
};

use super::{VoxelStoreSource, grid_store::ChunkOwnership, voxel_store_source::chunks};

#[derive(SystemParam)]
pub struct GridStoreEditApi<'w, 's> {
	sources: ResMut<'w, SourceManager>,
	messages: MessageWriter<'w, GridEditMessage>,
	grids: Query<'w, 's, (&'static Grid, &'static mut GridEditIdManager)>,
}

impl<'w, 's> GridStoreEditApi<'w, 's> {
	pub fn apply<E: GridEdit>(&mut self, grid: GridId, edit: E) -> Option<(GridEditId, GridGeneration)> {
		self.apply_boxed(grid, Box::new(edit)).ok()
	}

	pub fn apply_boxed(
		&mut self,
		grid: GridId,
		edit: Box<dyn GridEdit>,
	) -> Result<(GridEditId, GridGeneration), Box<dyn GridEdit>> {
		let (voxel_type, previous_generation, edit_id, generation) = {
			let Ok((grid_data, mut ids)) = self.grids.get_mut(grid) else { return Err(edit) };
			let previous_generation = ids.latest_generation();
			let (edit_id, generation) = ids.apply_edit();
			(grid_data.voxel_type_info(), previous_generation, edit_id, generation)
		};
		let edit = Arc::<dyn GridEdit>::from(edit);
		let affected_chunks = chunks_covering_nonzero_voxel_region(edit.affected_region());

		let source_id = self.sources.get_source::<VoxelStoreSource>()
			.expect("voxel store source was not registered")
			.source_id();
		let to_acquire: Vec<IVec3> = chunks(affected_chunks)
			.filter(|&chunk| {
				let store = self.sources.get_source::<VoxelStoreSource>().unwrap();
				store.chunk_ownership(grid, chunk) == ChunkOwnership::Unowned
			})
			.collect();

		let mut acquiring = Vec::new();
		for chunk in to_acquire {
			let region = NonZeroChunkRegion::from_single(chunk);
			let request_id = self.sources.request_voxels(grid, region, 0, Some(voxel_type.id), previous_generation);
			self.sources.transfer_ownership(source_id, grid, region);
			acquiring.push((chunk, request_id));
		}

		let store = self.sources.get_source_mut::<VoxelStoreSource>()
			.expect("voxel store source was not registered");
		for (chunk, request_id) in acquiring {
			store.begin_acquisition(grid, chunk, request_id, voxel_type, previous_generation);
		}
		for chunk in chunks(affected_chunks) {
			store.apply_edit_or_queue(grid, chunk, voxel_type, generation, edit.clone());
		}

		self.messages.write(GridEditMessage::from_shared(grid, generation, edit_id, edit));
		Ok((edit_id, generation))
	}
}
