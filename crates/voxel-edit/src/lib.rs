use std::collections::HashSet;

use bevy::ecs::change_detection::Mut;
use bevy::math::IVec3;
use bevy::prelude::*;

use voxel_data::grid::{reconcile_subgrids, Grid, GridId};
use voxel_data::subgrid::SubGrid;
use voxel_data::voxels::Voxel;

#[derive(Debug, Clone, Copy)]
enum GridEdit {
	Add { voxel_pos: IVec3, voxel: Voxel },
	Remove { voxel_pos: IVec3 },
}

impl GridEdit {
	fn voxel_pos(&self) -> IVec3 {
		match self {
			GridEdit::Add { voxel_pos, .. } | GridEdit::Remove { voxel_pos } => *voxel_pos,
		}
	}
	fn voxel(&self) -> Option<Voxel> {
		match self {
			GridEdit::Add { voxel, .. } => Some(*voxel),
			GridEdit::Remove { .. } => None,
		}
	}
}

#[derive(Component, Default, Debug)]
pub struct GridEdits {
	pending: Vec<GridEdit>,
}

impl GridEdits {
	pub fn add_voxel(&mut self, voxel_pos: &IVec3, voxel: &Voxel) {
		self.pending.push(GridEdit::Add { voxel_pos: *voxel_pos, voxel: *voxel });
	}

	pub fn remove_voxel(&mut self, voxel_pos: &IVec3) {
		self.pending.push(GridEdit::Remove { voxel_pos: *voxel_pos });
	}
}

#[bevy_trait_query::queryable]
pub trait EditGate {
	/// Return `false` to block this voxel from being written this frame.
	fn admit(&self, voxel_pos: IVec3) -> bool {
		let _ = voxel_pos;
		true
	}
	/// Called after a voxel was written.
	fn touched(&mut self, voxel_pos: IVec3) {
		let _ = voxel_pos;
	}
}

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ApplyGridEdits;

pub trait VoxelEditAppExt {
	fn register_edit_gate<T: EditGate + Component>(&mut self) -> &mut Self;
}

impl VoxelEditAppExt for App {
	fn register_edit_gate<T: EditGate + Component>(&mut self) -> &mut Self {
		use bevy_trait_query::RegisterExt;
		self.register_component_as::<dyn EditGate, T>();
		self
	}
}

#[derive(Default)]
pub struct VoxelEditPlugin;

impl Plugin for VoxelEditPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(PreUpdate, apply_grid_edits.in_set(ApplyGridEdits));
	}
}

pub fn apply_grid_edits(
	mut commands: Commands,
	mut grids: Query<(GridId, &mut Grid, &mut GridEdits)>,
	mut gates: Query<&mut dyn EditGate>,
	mut sub_grids: Query<&mut SubGrid>,
) {
	for (grid_entity, mut grid, mut edits) in grids.iter_mut() {
		if edits.pending.is_empty() { continue; }

		// Collected once so each voxel can check (via &) then notify (via &mut).
		let mut gate_list: Vec<Mut<dyn EditGate>> =
			gates.get_mut(grid_entity).map(|w| w.into_iter().collect()).unwrap_or_default();

		let mut touched: HashSet<IVec3> = HashSet::new();
		for edit in std::mem::take(&mut edits.pending) {
			let pos = edit.voxel_pos();
			if !gate_list.iter().all(|gate| gate.admit(pos)) { continue; }

			touched.insert(grid.set_voxel(pos, edit.voxel()));
			for gate in gate_list.iter_mut() {
				gate.touched(pos);
			}
		}

		reconcile_subgrids(grid_entity, grid.as_mut(), touched, &mut commands, &mut sub_grids);
	}
}
