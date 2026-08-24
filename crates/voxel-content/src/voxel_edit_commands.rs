use std::sync::Arc;

use bevy::{ecs::system::SystemParam, prelude::*};
use voxel_data::grid::{Grid, GridId};
use voxel_sources::{
	SourceManager,
	edit::{GridEdit, GridEditId, GridEditIdManager, GridEditMessage, GridGeneration},
};

use crate::VoxelStoreSource;

#[derive(SystemParam)]
pub struct VoxelEditCommands<'w, 's> {
	sources: ResMut<'w, SourceManager>,
	messages: MessageWriter<'w, GridEditMessage>,
	grids: Query<'w, 's, (&'static Grid, &'static mut GridEditIdManager)>,
	store: Res<'w, VoxelStoreSource>,
}

impl<'w, 's> VoxelEditCommands<'w, 's> {
	pub fn apply<E: GridEdit>(&mut self, grid: GridId, edit: E) -> Option<(GridEditId, GridGeneration)> {
		self.apply_boxed(grid, Box::new(edit)).ok()
	}

	pub fn apply_boxed(
		&mut self,
		grid: GridId,
		edit: Box<dyn GridEdit>,
	) -> Result<(GridEditId, GridGeneration), Box<dyn GridEdit>> {
		let (voxel_type, previous_generation, edit_id, generation) = {
			let Ok((grid, mut ids)) = self.grids.get_mut(grid) else { return Err(edit) };
			let previous_generation = ids.latest_generation();
			let (edit_id, generation) = ids.apply_edit();
			(grid.voxel_type_info(), previous_generation, edit_id, generation)
		};
		let edit = Arc::<dyn GridEdit>::from(edit);
		self.store.apply_edit(
			&mut self.sources,
			grid,
			voxel_type,
			previous_generation,
			generation,
			edit.clone(),
		);
		self.messages.write(GridEditMessage::from_shared(grid, generation, edit_id, edit));
		Ok((edit_id, generation))
	}
}
