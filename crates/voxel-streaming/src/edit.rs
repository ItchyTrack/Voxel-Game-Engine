use std::sync::Arc;

use bevy::prelude::*;
use voxel_data::region::NonZeroVoxelRegion;
use voxel_data::sdf::Sdf;
use voxel_data::voxels::Voxel;
use voxel_edit::GridEdit;

use crate::GridStreaming;

struct PendingGridEdit {
	edit: GridEdit,
}

#[derive(Component, Default)]
pub struct GridEdits {
	pending: Vec<PendingGridEdit>,
}

impl GridEdits {
	fn push_pending(&mut self, edit: GridEdit) {
		self.pending.push(PendingGridEdit { edit });
	}

	pub fn add_voxel(&mut self, voxel_pos: IVec3, voxel: Voxel) {
		self.push_pending(GridEdit::AddArea { region: NonZeroVoxelRegion::from_single(voxel_pos), voxel } );
	}

	pub fn remove_voxel(&mut self, voxel_pos: IVec3) {
		self.push_pending(GridEdit::RemoveArea { region: NonZeroVoxelRegion::from_single(voxel_pos) });
	}

	pub fn add_area(&mut self, region: NonZeroVoxelRegion, voxel: Voxel) {
		self.push_pending(GridEdit::AddArea { region, voxel });
	}

	pub fn remove_area(&mut self, region: NonZeroVoxelRegion) {
		self.push_pending(GridEdit::RemoveArea { region });
	}

	pub fn apply_sdf(&mut self, bounds_min: Vec3, bounds_max: Vec3, voxel: Voxel, sdf: Arc<dyn Sdf>) {
		self.push_pending(GridEdit::ApplySdf { bounds_min, bounds_max, face_resolution: IVec2::splat(9), iterations: 6, voxel, sdf });
	}

	pub fn clear_sdf(&mut self, bounds_min: Vec3, bounds_max: Vec3, sdf: Arc<dyn Sdf>) {
		self.push_pending(GridEdit::ClearSdf { bounds_min, bounds_max, face_resolution: IVec2::splat(9), iterations: 6, sdf });
	}
}

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ApplyGridEdits;

pub fn apply_grid_edits(
	mut grids: Query<(&mut GridEdits, &mut GridStreaming)>,
) {
	for (mut edits, mut streaming) in &mut grids {
		for PendingGridEdit { edit } in std::mem::take(&mut edits.pending) {
			streaming.pending_take_edits.extend(edit.into_resolved());
		}
	}
}
