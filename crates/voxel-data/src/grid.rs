use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use bevy::math::{I16Vec3, IVec3, Quat, Vec3};

use crate::subgrid::{SubGrid, SubGridId, SubGridRef};
use crate::voxels::{Voxel, Voxels};

pub const SUB_GRID_SIZE: i32 = 64;

/// A [`Grid`] entity. Each rigid voxel object owns one.
pub type GridId = Entity;

#[derive(Debug, Clone, Copy)]
enum GridEdit {
	Add { voxel_pos: IVec3, voxel: Voxel },
	Remove { voxel_pos: IVec3 },
}

#[derive(Debug)]
struct SubGridSlot {
	voxels: Voxels,
	entity: SubGridId,
}

#[derive(Debug, Component, Default)]
pub struct Grid {
	subgrids: HashMap<IVec3, SubGridSlot>,
	pending: Vec<GridEdit>,
}

impl Grid {
	pub fn new() -> Self { Self::default() }

	fn sub_grid_pos_of(pos: &IVec3) -> IVec3 {
		pos.div_euclid(IVec3::splat(SUB_GRID_SIZE)) * SUB_GRID_SIZE
	}
	fn local_of(pos: &IVec3) -> I16Vec3 {
		pos.rem_euclid(IVec3::splat(SUB_GRID_SIZE)).as_i16vec3()
	}

	pub fn add_voxel(&mut self, voxel_pos: &IVec3, voxel: &Voxel) {
		self.pending.push(GridEdit::Add { voxel_pos: *voxel_pos, voxel: *voxel });
	}
	pub fn remove_voxel(&mut self, voxel_pos: &IVec3) {
		self.pending.push(GridEdit::Remove { voxel_pos: *voxel_pos });
	}

	pub fn voxel(&self, voxel_pos: &IVec3) -> Option<&Voxel> {
		self.subgrids
			.get(&Self::sub_grid_pos_of(voxel_pos))?
			.voxels
			.voxel(&Self::local_of(voxel_pos))
	}

	pub fn view(&self, sub_grid: &SubGrid) -> Option<SubGridRef<'_>> {
		let slot = self.subgrids.get(&sub_grid.sub_grid_pos())?;
		Some(SubGridRef::new(&slot.voxels, sub_grid.sub_grid_pos()))
	}

	pub fn subgrids(&self) -> impl Iterator<Item = SubGridRef<'_>> {
		self.subgrids.iter().map(|(pos, slot)| SubGridRef::new(&slot.voxels, *pos))
	}

	pub fn raycast(&self, origin: Vec3, dir: Vec3) -> Option<GridRaycastHit> {
		self.subgrids()
			.filter_map(|sub_grid| {
				let sub_origin = sub_grid.sub_grid_pos().as_vec3();
				raycast_sub_grid(sub_grid, origin - sub_origin, dir)
					.map(|(hit_local, normal_local, distance)| GridRaycastHit {
						voxel_pos: sub_grid.sub_grid_pos() + hit_local.as_ivec3(),
						normal: normal_local.as_ivec3(),
						distance,
					})
			})
			.min_by(|a, b| a.distance.total_cmp(&b.distance))
	}
}

pub struct GridRaycastHit {
	pub voxel_pos: IVec3,
	pub normal: IVec3,
	pub distance: f32,
}

fn raycast_sub_grid(sub_grid: SubGridRef, origin: Vec3, dir: Vec3) -> Option<(I16Vec3, bevy::math::I8Vec3, f32)> {
	// GridTree::raycast takes a Transform whose rotation maps +Z to the ray dir.
	let transform = Transform {
		translation: origin,
		rotation: Quat::from_rotation_arc(Vec3::Z, dir),
		scale: Vec3::ONE,
	};
	sub_grid.voxels().grid_tree().raycast(&transform, None)
}

pub fn apply_grid_edits(
	mut commands: Commands,
	mut grids: Query<(Entity, &mut Grid)>,
	mut sub_grids: Query<&mut SubGrid>,
) {
	for (grid_entity, mut grid) in grids.iter_mut() {
		if grid.pending.is_empty() { continue; }
		let edits = std::mem::take(&mut grid.pending);
		let mut touched: HashSet<IVec3> = HashSet::new();

		for edit in edits {
			match edit {
				GridEdit::Add { voxel_pos, voxel } => {
					let pos = Grid::sub_grid_pos_of(&voxel_pos);
					let local = Grid::local_of(&voxel_pos);
					grid.subgrids
						.entry(pos)
						.or_insert_with(|| SubGridSlot { voxels: Voxels::new(), entity: Entity::PLACEHOLDER })
						.voxels
						.add_voxel(local, voxel);
					touched.insert(pos);
				}
				GridEdit::Remove { voxel_pos } => {
					let pos = Grid::sub_grid_pos_of(&voxel_pos);
					let local = Grid::local_of(&voxel_pos);
					if let Some(slot) = grid.subgrids.get_mut(&pos) {
						slot.voxels.remove_voxel(&local);
						touched.insert(pos);
					}
				}
			}
		}

		for pos in touched {
			let Some(slot) = grid.subgrids.get(&pos) else { continue };
			let entity = slot.entity;
			if slot.voxels.is_empty() {
				if entity != Entity::PLACEHOLDER { commands.entity(entity).despawn(); }
				grid.subgrids.remove(&pos);
			} else if entity == Entity::PLACEHOLDER {
				let entity = commands.spawn((SubGrid::new(grid_entity, pos), ChildOf(grid_entity))).id();
				grid.subgrids.get_mut(&pos).unwrap().entity = entity;
			} else if let Ok(mut sub) = sub_grids.get_mut(entity) {
				sub.set_changed();
			}
		}
	}
}
