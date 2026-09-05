use std::sync::Arc;

use bevy::{ecs::system::SystemParam, math::IVec3, prelude::*};
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, chunk_origin, chunks_covering_nonzero_voxel_region};
use voxel_data::grid::{Grid, GridId};
use voxel_mass::{
	GridEditMassReaders, GridMassProperties, MassError, VoxelMassReaders,
	edit_reservation_error,
};
use voxel_sources::{
	SourceManager,
	edit::{GridEdit, GridEditId, GridEditIdManager, GridEditMessage, GridGeneration},
};
use voxel_trees::region::NonZeroVoxelRegion;

use super::{VoxelStoreSource, grid_store::ChunkOwnership, voxel_store_source::chunks};

#[derive(SystemParam)]
pub struct GridStoreEditApi<'w, 's> {
	sources: ResMut<'w, SourceManager>,
	messages: MessageWriter<'w, GridEditMessage>,
	edit_mass_readers: Res<'w, GridEditMassReaders>,
	voxel_mass_readers: Res<'w, VoxelMassReaders>,
	grids: Query<'w, 's, (&'static Grid, &'static mut GridEditIdManager, Option<&'static GridMassProperties>)>,
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
		let (voxel_type, current_mass) = {
			let Ok((grid_data, _, mass)) = self.grids.get_mut(grid) else { return Err(edit) };
			(grid_data.voxel_type_info(), mass.cloned().unwrap_or_default())
		};
		let affected_region = edit.affected_region();
		let affected_chunks = chunks_covering_nonzero_voxel_region(affected_region);
		let mass_bearing = self.voxel_mass_readers.contains(voxel_type.id);

		let source = self.sources.get_source::<VoxelStoreSource>()
			.expect("voxel store source was not registered");
		let chunk_ownership: Vec<_> = chunks(affected_chunks)
			.map(|chunk| (chunk, source.chunk_ownership(grid, chunk)))
			.collect();

		if mass_bearing {
			let first_chunk = chunk_ownership.first().expect("an edit must affect at least one chunk").0;
			let clipped = affected_region.intersection(chunk_voxel_region(first_chunk)).unwrap();
			assert!(
				self.edit_mass_readers.read(edit.as_ref(), clipped, &self.voxel_mass_readers).is_some(),
				"mass-bearing grid edit has no registered mass reader",
			);
		}

		let reservations: Vec<_> = chunk_ownership.iter().map(|(chunk, ownership)| {
			if !mass_bearing || *ownership == ChunkOwnership::Owned {
				return MassError::ZERO;
			}
			let clipped = affected_region.intersection(chunk_voxel_region(*chunk)).unwrap();
			let new_mass = self.edit_mass_readers
				.read(edit.as_ref(), clipped, &self.voxel_mass_readers)
				.expect("mass-bearing grid edit has no registered mass reader");
			edit_reservation_error(
				*current_mass.nominal(),
				current_mass.aggregate_error(),
				new_mass,
				clipped,
			)
		}).collect();

		let (previous_generation, edit_id, generation) = {
			let (_, mut ids, _) = self.grids.get_mut(grid).unwrap();
			let previous_generation = ids.latest_generation();
			let (edit_id, generation) = ids.apply_edit();
			(previous_generation, edit_id, generation)
		};
		let edit = Arc::<dyn GridEdit>::from(edit);
		let source_id = source.source_id();

		let mut acquiring = Vec::new();
		for (chunk, ownership) in &chunk_ownership {
			if *ownership != ChunkOwnership::Unowned { continue; }
			let region = NonZeroChunkRegion::from_single(*chunk);
			let request_id = self.sources.request_voxels(grid, region, 0, Some(voxel_type.id), previous_generation);
			self.sources.transfer_ownership(source_id, grid, region);
			acquiring.push((*chunk, request_id));
		}

		let store = self.sources.get_source_mut::<VoxelStoreSource>()
			.expect("voxel store source was not registered");
		store.set_mass_readers(self.voxel_mass_readers.clone());
		for (chunk, request_id) in acquiring {
			store.begin_acquisition(grid, chunk, request_id, voxel_type, previous_generation);
		}
		for ((chunk, _), reserved_error) in chunk_ownership.into_iter().zip(reservations) {
			store.apply_edit_or_queue(grid, chunk, voxel_type, generation, edit.clone(), reserved_error);
		}

		self.messages.write(GridEditMessage::from_shared(grid, generation, edit_id, edit));
		Ok((edit_id, generation))
	}
}

fn chunk_voxel_region(chunk: IVec3) -> NonZeroVoxelRegion {
	NonZeroVoxelRegion::from_min_size(chunk_origin(chunk), UVec3::splat(CHUNK_SIZE)).unwrap()
}
