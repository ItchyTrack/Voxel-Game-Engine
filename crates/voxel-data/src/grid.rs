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
	RemoveArea { min: IVec3, size: IVec3 },
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

	pub fn add_area(&mut self, min: &IVec3, size: &IVec3, voxel: &Voxel) {
		for x in 0..size.x {
			for y in 0..size.y {
				for z in 0..size.z {
					self.add_voxel(&(min + IVec3::new(x, y, z)), voxel);
				}
			}
		}
	}

	pub fn remove_area(&mut self, min: &IVec3, size: &IVec3) {
		self.pending.push(GridEdit::RemoveArea { min: *min, size: *size });
	}

	/// Clear `[min, min + size)` directly, marking touched sub-grids. Sub-grids
	/// fully inside the box are dropped wholesale; partial ones do a bulk tree
	/// removal of the clamped slice.
	fn clear_area(&mut self, min: IVec3, size: IVec3, touched: &mut HashSet<IVec3>) {
		if size.cmple(IVec3::ZERO).any() { return; }
		let sub = IVec3::splat(SUB_GRID_SIZE);
		let hi = min + size;
		let sg_lo = min.div_euclid(sub);
		let sg_hi = (hi - IVec3::ONE).div_euclid(sub);
		for sx in sg_lo.x..=sg_hi.x {
			for sy in sg_lo.y..=sg_hi.y {
				for sz in sg_lo.z..=sg_hi.z {
					let sub_origin = IVec3::new(sx, sy, sz) * SUB_GRID_SIZE;
					let Some(slot) = self.subgrids.get_mut(&sub_origin) else { continue };
					let cell_lo = min.max(sub_origin);
					let cell_hi = hi.min(sub_origin + sub);
					let local = cell_lo - sub_origin;
					let extent = cell_hi - cell_lo;
					if local == IVec3::ZERO && extent == sub {
						slot.voxels = Voxels::new();
					} else {
						slot.voxels.remove_area(local.as_i16vec3(), extent.as_i16vec3());
					}
					touched.insert(sub_origin);
				}
			}
		}
	}

	/// Write every voxel of `src` directly into this grid at offset `base`,
	/// bypassing the pending queue. Cells are bulk-filled and split across
	/// sub-grid boundaries. Returns the touched origins for [`reconcile_subgrids`].
	pub fn splat_voxels(&mut self, base: IVec3, src: &Voxels) -> HashSet<IVec3> {
		let mut touched = HashSet::new();
		let palette = src.palette();
		let sub = IVec3::splat(SUB_GRID_SIZE);
		for (pos, size, palette_id) in src.grid_tree().iter() {
			let Some(voxel) = palette.voxel(palette_id) else { continue };
			let voxel = *voxel;
			let lo = base + pos.as_ivec3();
			let hi = lo + IVec3::splat(size as i32);
			let sg_lo = lo.div_euclid(sub);
			let sg_hi = (hi - IVec3::ONE).div_euclid(sub);
			for sx in sg_lo.x..=sg_hi.x {
				for sy in sg_lo.y..=sg_hi.y {
					for sz in sg_lo.z..=sg_hi.z {
						let sub_origin = IVec3::new(sx, sy, sz) * SUB_GRID_SIZE;
						let cell_lo = lo.max(sub_origin);
						let cell_hi = hi.min(sub_origin + sub);
						let local = (cell_lo - sub_origin).as_i16vec3();
						let extent = (cell_hi - cell_lo).as_i16vec3();
						self.subgrids
							.entry(sub_origin)
							.or_insert_with(|| SubGridSlot { voxels: Voxels::new(), entity: Entity::PLACEHOLDER })
							.voxels
							.add_area(local, extent, voxel);
						touched.insert(sub_origin);
					}
				}
			}
		}
		touched
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
				GridEdit::RemoveArea { min, size } => grid.clear_area(min, size, &mut touched),
			}
		}

		reconcile_subgrids(grid_entity, grid.as_mut(), touched, &mut commands, &mut sub_grids);
	}
}

pub fn reconcile_subgrids(
	grid_entity: Entity,
	grid: &mut Grid,
	touched: impl IntoIterator<Item = IVec3>,
	commands: &mut Commands,
	sub_grids: &mut Query<&mut SubGrid>,
) {
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

#[cfg(test)]
mod tests {
	use super::*;
	use bevy::math::I16Vec3;

	fn vox(c: u8) -> Voxel { Voxel { color: [c, c, c, 255], mass: 1 } }

	#[test]
	fn splat_voxels_reproduces_source() {
		// Negative, non-aligned base so cells straddle sub-grid boundaries.
		let mut src = Voxels::new();
		for x in 0..40 { for y in 0..40 { for z in 0..40 {
			src.add_voxel(I16Vec3::new(x, y, z), vox(1));
		}}}
		for &(x, y, z, c) in &[(5i16, 60i16, 3i16, 2u8), (70, 1, 70, 3), (33, 33, 33, 4)] {
			src.add_voxel(I16Vec3::new(x, y, z), vox(c));
		}

		let base = IVec3::new(-17, 5, -70);
		let mut grid = Grid::new();
		grid.splat_voxels(base, &src);

		let mut count = 0u64;
		for (pos, size, id) in src.grid_tree().iter() {
			let voxel = *src.palette().voxel(id).unwrap();
			for dx in 0..size as i32 { for dy in 0..size as i32 { for dz in 0..size as i32 {
				let p = base + pos.as_ivec3() + IVec3::new(dx, dy, dz);
				assert_eq!(grid.voxel(&p), Some(&voxel), "mismatch at {p:?}");
				count += 1;
			}}}
		}
		let total: u64 = grid.subgrids.values().map(|s| s.voxels.grid_tree().len()).sum();
		assert_eq!(total, count);
	}

	#[test]
	fn clear_area_drops_full_and_slices_partial() {
		let mut grid = Grid::new();
		// Fill a 64-wide region spanning 2x2x2 sub-grids, plus one voxel outside.
		grid.add_area(&IVec3::ZERO, &IVec3::splat(64), &vox(1));
		grid.add_voxel(&IVec3::new(100, 0, 0), &vox(2));
		for e in std::mem::take(&mut grid.pending) {
			if let GridEdit::Add { voxel_pos, voxel } = e {
				let p = Grid::sub_grid_pos_of(&voxel_pos);
				grid.subgrids.entry(p).or_insert_with(|| SubGridSlot { voxels: Voxels::new(), entity: Entity::PLACEHOLDER })
					.voxels.add_voxel(Grid::local_of(&voxel_pos), voxel);
			}
		}

		let mut touched = HashSet::new();
		grid.clear_area(IVec3::splat(16), IVec3::splat(48), &mut touched);

		assert_eq!(grid.voxel(&IVec3::splat(40)), None, "fully-covered region cleared");
		assert_eq!(grid.voxel(&IVec3::splat(20)), None, "partial slice cleared");
		assert_eq!(grid.voxel(&IVec3::splat(8)), Some(&vox(1)), "untouched corner survives");
		assert_eq!(grid.voxel(&IVec3::new(100, 0, 0)), Some(&vox(2)), "outside area untouched");
	}
}
