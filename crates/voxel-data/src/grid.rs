use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use bevy::math::{I16Vec3, IVec3};

use crate::subgrid::{SubGrid, SubGridId};
use crate::voxels::Voxel;

pub const CHUNK_SIZE: i32 = 64;

/// A [`Grid`] entity. Each rigid voxel object owns one.
pub type GridId = Entity;

#[derive(Debug, Clone, Copy)]
enum GridEdit {
	Add { voxel_pos: IVec3, voxel: Voxel },
	Remove { voxel_pos: IVec3 },
}

#[derive(Debug, Component, Default)]
pub struct Grid {
	chunk_to_entity: HashMap<IVec3, SubGridId>,
	pending: Vec<GridEdit>,
}

impl Grid {
	pub fn new() -> Self { Self::default() }

	fn chunk_of(pos: &IVec3) -> IVec3 {
		pos.div_euclid(IVec3::splat(CHUNK_SIZE))
	}
	fn local_of(pos: &IVec3) -> I16Vec3 {
		pos.rem_euclid(IVec3::splat(CHUNK_SIZE)).as_i16vec3()
	}

	pub fn add_voxel(&mut self, voxel_pos: &IVec3, voxel: &Voxel) {
		self.pending.push(GridEdit::Add { voxel_pos: *voxel_pos, voxel: *voxel });
	}
	pub fn remove_voxel(&mut self, voxel_pos: &IVec3) {
		self.pending.push(GridEdit::Remove { voxel_pos: *voxel_pos });
	}
}

pub fn apply_grid_edits(
	mut commands: Commands,
	mut grids: Query<(Entity, &mut Grid)>,
	mut sub_grids: Query<&mut SubGrid>,
) {
	for (grid_entity, mut grid) in grids.iter_mut() {
		if grid.pending.is_empty() { continue; }
		let edits = std::mem::take(&mut grid.pending);

		// New sub-grids are accumulated per chunk and only spawned (assigning the
		// entity that *is* their `SubGridId`) once known to be non-empty.
		let mut new_sub_grids: HashMap<IVec3, SubGrid> = HashMap::new();
		let mut touched: HashSet<SubGridId> = HashSet::new();

		for edit in edits {
			match edit {
				GridEdit::Add { voxel_pos, voxel } => {
					let chunk = Grid::chunk_of(&voxel_pos);
					let local = Grid::local_of(&voxel_pos);
					if let Some(&entity) = grid.chunk_to_entity.get(&chunk) {
						if let Ok(mut sub) = sub_grids.get_mut(entity) {
							sub.add_voxel(local, voxel);
							touched.insert(entity);
						}
					} else {
						new_sub_grids
							.entry(chunk)
							.or_insert_with(|| SubGrid::new(grid_entity, chunk * CHUNK_SIZE))
							.add_voxel(local, voxel);
					}
				}
				GridEdit::Remove { voxel_pos } => {
					let chunk = Grid::chunk_of(&voxel_pos);
					let local = Grid::local_of(&voxel_pos);
					if let Some(&entity) = grid.chunk_to_entity.get(&chunk) {
						if let Ok(mut sub) = sub_grids.get_mut(entity) {
							sub.remove_voxel(&local);
							touched.insert(entity);
						}
					} else if let Some(sub) = new_sub_grids.get_mut(&chunk) {
						sub.remove_voxel(&local);
					}
				}
			}
		}

		for (chunk, sub) in new_sub_grids {
			if sub.is_empty() { continue; }
			let entity = commands.spawn((sub, ChildOf(grid_entity))).id();
			grid.chunk_to_entity.insert(chunk, entity);
		}

		for entity in touched {
			let empty = sub_grids.get(entity).map(|s| s.is_empty()).unwrap_or(true);
			if empty {
				commands.entity(entity).despawn();
				grid.chunk_to_entity.retain(|_, v| *v != entity);
			}
		}
	}
}
