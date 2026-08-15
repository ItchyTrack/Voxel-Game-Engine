use std::collections::HashSet;
use std::sync::Arc;

use bevy::prelude::*;
use tracy_client::span;
use voxel_data::grid::{Grid, GridId, reconcile_subgrids};
use voxel_data::region::NonZeroVoxelRegion;
use voxel_data::sdf::Sdf;
use voxel_data::subgrid::SubGrid;
use voxel_data::voxels::Voxel;
use voxel_edit::{GridEdit, apply_grid_edit};

use crate::GridStreaming;

struct PendingGridEdit {
	edit: GridEdit,
	publish: bool,
}

#[derive(Component, Default)]
pub struct GridEdits {
	pending: Vec<PendingGridEdit>,
}

impl GridEdits {
	fn push_pending(&mut self, edit: GridEdit, publish: bool) {
		self.pending.push(PendingGridEdit { edit, publish });
	}

	pub fn add_voxel(&mut self, voxel_pos: &IVec3, voxel: Voxel) {
		self.push_pending(GridEdit::Add { voxel_pos: *voxel_pos, voxel }, true);
	}

	pub fn remove_voxel(&mut self, voxel_pos: &IVec3) {
		self.push_pending(GridEdit::Remove { voxel_pos: *voxel_pos }, true);
	}

	pub fn add_area(&mut self, region: NonZeroVoxelRegion, voxel: Voxel) {
		self.push_pending(GridEdit::AddArea { region, voxel }, true);
	}

	pub fn remove_area(&mut self, region: NonZeroVoxelRegion) {
		self.push_pending(GridEdit::RemoveArea { region }, true);
	}

	pub fn apply_sdf(&mut self, bounds_min: Vec3, bounds_max: Vec3, voxel: Voxel, sdf: Arc<dyn Sdf>) {
		self.push_pending(GridEdit::ApplySdf { bounds_min, bounds_max, face_resolution: IVec2::splat(9), iterations: 6, voxel, sdf }, true);
	}

	pub fn clear_sdf(&mut self, bounds_min: Vec3, bounds_max: Vec3, sdf: Arc<dyn Sdf>) {
		self.push_pending(GridEdit::ClearSdf { bounds_min, bounds_max, face_resolution: IVec2::splat(9), iterations: 6, sdf }, true);
	}

	/// Enqueue a local edit for streaming ownership and authoritative publication.
	pub fn push_edit(&mut self, edit: GridEdit) {
		self.push_pending(edit, true);
	}

	/// Materialize an already-authoritative command without publishing it again.
	pub fn push_authoritative_edit(&mut self, edit: GridEdit) {
		self.push_pending(edit, false);
	}
}

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ApplyGridEdits;

impl GridStreaming {
	pub(crate) fn replay_stalled(&mut self, chunk: IVec3, edits: &mut Option<Mut<GridEdits>>) {
		let _zone = span!();
		let Some(stalled) = self.stalled_edits.remove(&chunk) else { return; };
		if let Some(edits) = edits.as_mut() {
			for edit in stalled {
				edits.push_edit(edit);
			}
		}
		if self.stalled_pinned.remove(&chunk) {
			self.release_completed(chunk);
		}
	}

	fn admit_edit(&mut self, edit: &GridEdit) -> bool {
		self.pending_take_edits.extend(edit.resolved_commands());
		false
	}
}

pub fn apply_grid_edits(
	mut commands: Commands,
	mut grids: Query<(GridId, &mut Grid, &mut GridEdits, &mut GridStreaming)>,
	mut sub_grids: Query<&mut SubGrid>,
) {
	for (grid_entity, mut grid, mut edits, mut streaming) in grids.iter_mut() {
		if edits.pending.is_empty() { continue; }

		let mut touched: HashSet<IVec3> = HashSet::new();
		for pending in std::mem::take(&mut edits.pending) {
			let PendingGridEdit { edit, publish } = pending;
			if publish && !streaming.admit_edit(&edit) { continue; }
			touched.extend(apply_grid_edit(grid.as_mut(), &edit));
		}

		reconcile_subgrids(grid_entity, grid.as_mut(), touched, &mut commands, &mut sub_grids);
	}
}
