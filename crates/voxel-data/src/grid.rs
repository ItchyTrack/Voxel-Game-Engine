use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};

use bevy::ecs::storage::SparseSetIndex;
use bevy::prelude::*;
use bevy::math::{I16Vec3, IVec3, Vec3};

use crate::voxels::{Voxel, Voxels};

pub const CHUNK_SIZE: i32 = 64;

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct SubGridId(pub u32);

impl SparseSetIndex for SubGridId {
	fn sparse_set_index(&self) -> usize { self.0 as usize }
	fn get_sparse_set_index(index: usize) -> Self { Self(index as u32) }
}

#[derive(Debug, Component)]
pub struct SubGrid {
	voxels: Voxels,
	grid: Entity,
	id: SubGridId,
	sub_grid_pos: IVec3,
	reupload: AtomicBool,
}

impl SubGrid {
	fn new(grid: Entity, id: SubGridId, sub_grid_pos: IVec3) -> Self {
		Self {
			voxels: Voxels::new(),
			grid,
			id,
			sub_grid_pos,
			reupload: AtomicBool::new(true),
		}
	}

	pub fn grid(&self) -> Entity { self.grid }
	pub fn id(&self) -> SubGridId { self.id }
	pub fn sub_grid_pos(&self) -> IVec3 { self.sub_grid_pos }
	pub fn get_voxels(&self) -> &Voxels { &self.voxels }
	pub fn get_voxel(&self, pos: &I16Vec3) -> Option<&Voxel> { self.voxels.get_voxel(pos) }

	fn add_voxel(&mut self, pos: I16Vec3, voxel: Voxel) -> Option<Voxel> {
		self.reupload.store(true, Ordering::Release);
		self.voxels.add_voxel(pos, voxel)
	}
	fn remove_voxel(&mut self, pos: &I16Vec3) -> Option<Voxel> {
		self.reupload.store(true, Ordering::Release);
		self.voxels.remove_voxel(pos)
	}

	pub fn is_empty(&self) -> bool {
		self.voxels.get_voxels().len() == 0
	}

	pub fn reupload(&self) -> bool {
		self.reupload.load(Ordering::Acquire)
	}
	pub fn clear_reupload(&self) {
		self.reupload.store(false, Ordering::Release);
	}

	pub fn aabb(&self, transform: &Transform) -> Option<(Vec3, Vec3)> {
		let (min, max) = self.voxels.get_bounding_box()?;
		let min = min.as_vec3();
		let max = max.as_vec3() + Vec3::new(1.0, 1.0, 1.0);
		let corners = [
			min,
			Vec3::new(max.x, min.y, min.z),
			Vec3::new(min.x, max.y, min.z),
			Vec3::new(min.x, min.y, max.z),
			Vec3::new(max.x, max.y, min.z),
			Vec3::new(max.x, min.y, max.z),
			Vec3::new(min.x, max.y, max.z),
			max,
		];
		let rotated_corners = corners.map(|c| *transform * c);
		Some((
			rotated_corners.iter().fold(Vec3::splat(f32::MAX), |acc, c| acc.min(*c)),
			rotated_corners.iter().fold(Vec3::splat(f32::MIN), |acc, c| acc.max(*c)),
		))
	}
}

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct GridId(pub usize);

#[derive(Debug, Clone, Copy)]
enum GridEdit {
	Add { voxel_pos: IVec3, voxel: Voxel },
	Remove { voxel_pos: IVec3 },
}

#[derive(Debug, Component, Default)]
pub struct Grid {
	chunk_to_id: HashMap<IVec3, SubGridId>,
	id_to_entity: HashMap<SubGridId, Entity>,
	next_id: u32,
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

	pub fn sub_grid_entity(&self, id: SubGridId) -> Option<Entity> {
		self.id_to_entity.get(&id).copied()
	}
	pub fn sub_grid_entities(&self) -> impl Iterator<Item = (SubGridId, Entity)> + '_ {
		self.id_to_entity.iter().map(|(id, e)| (*id, *e))
	}

	fn id_for_chunk(&mut self, chunk: IVec3) -> SubGridId {
		if let Some(id) = self.chunk_to_id.get(&chunk) {
			return *id;
		}
		let id = SubGridId(self.next_id);
		self.next_id += 1;
		self.chunk_to_id.insert(chunk, id);
		id
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

		let mut new_sub_grids: HashMap<SubGridId, SubGrid> = HashMap::new();
		let mut touched: HashSet<SubGridId> = HashSet::new();

		for edit in edits {
			match edit {
				GridEdit::Add { voxel_pos, voxel } => {
					let chunk = Grid::chunk_of(&voxel_pos);
					let local = Grid::local_of(&voxel_pos);
					let id = grid.id_for_chunk(chunk);
					if let Some(&entity) = grid.id_to_entity.get(&id) {
						if let Ok(mut sub) = sub_grids.get_mut(entity) {
							sub.add_voxel(local, voxel);
							touched.insert(id);
						}
					} else {
						new_sub_grids
							.entry(id)
							.or_insert_with(|| SubGrid::new(grid_entity, id, chunk * CHUNK_SIZE))
							.add_voxel(local, voxel);
					}
				}
				GridEdit::Remove { voxel_pos } => {
					let chunk = Grid::chunk_of(&voxel_pos);
					let local = Grid::local_of(&voxel_pos);
					let Some(&id) = grid.chunk_to_id.get(&chunk) else { continue };
					if let Some(&entity) = grid.id_to_entity.get(&id) {
						if let Ok(mut sub) = sub_grids.get_mut(entity) {
							sub.remove_voxel(&local);
							touched.insert(id);
						}
					} else if let Some(sub) = new_sub_grids.get_mut(&id) {
						sub.remove_voxel(&local);
					}
				}
			}
		}

		for (id, sub) in new_sub_grids {
			if sub.is_empty() {
				grid.chunk_to_id.retain(|_, v| *v != id);
				continue;
			}
			let entity = commands.spawn((sub, ChildOf(grid_entity))).id();
			grid.id_to_entity.insert(id, entity);
		}

		for id in touched {
			let Some(&entity) = grid.id_to_entity.get(&id) else { continue };
			let empty = sub_grids.get(entity).map(|s| s.is_empty()).unwrap_or(true);
			if empty {
				commands.entity(entity).despawn();
				grid.id_to_entity.remove(&id);
				grid.chunk_to_id.retain(|_, v| *v != id);
			}
		}
	}
}
