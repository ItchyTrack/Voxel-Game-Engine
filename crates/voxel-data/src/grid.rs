use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use bevy::math::{I16Vec3, IVec3, Quat, Vec3};

use tracy_client::span;

use crate::subgrid::{SubGrid, SubGridId, SubGridRef};
use crate::voxels::{Voxel, Voxels};

// Temporary fixed sub-grid extent. Do not rely on sub-grids staying aligned to
// this grid: future physics/rendering work is expected to choose more optimal
// sub-grid placements dynamically, so callers should treat sub-grid origins as
// implementation details rather than stable chunk/grid boundaries.
pub const SUB_GRID_SIZE: i32 = 57;

/// A [`Grid`] entity. Each rigid voxel object owns one.
pub type GridId = Entity;

#[derive(Debug)]
pub(crate) struct SubGridSlot {
	pub(crate) voxels: Voxels,
	pub(crate) entity: SubGridId,
}

#[derive(Debug, Component, Default)]
pub struct Grid {
	pub(crate) subgrids: HashMap<IVec3, SubGridSlot>,
}

impl Grid {
	pub fn new() -> Self { Self::default() }

	fn sub_grid_pos_of(pos: &IVec3) -> IVec3 {
		pos.div_euclid(IVec3::splat(SUB_GRID_SIZE)) * SUB_GRID_SIZE
	}
	fn local_of(pos: &IVec3) -> I16Vec3 {
		pos.rem_euclid(IVec3::splat(SUB_GRID_SIZE)).as_i16vec3()
	}

	/// Write (`Some`) or remove (`None`) a single voxel. Returns the touched
	/// sub-grid origin for [`reconcile_subgrids`].
	pub fn set_voxel(&mut self, voxel_pos: IVec3, voxel: Option<Voxel>) -> IVec3 {
		let pos = Self::sub_grid_pos_of(&voxel_pos);
		let local = Self::local_of(&voxel_pos);
		match voxel {
			Some(voxel) => {
				self.subgrids
					.entry(pos)
					.or_insert_with(|| SubGridSlot { voxels: Voxels::new(), entity: Entity::PLACEHOLDER })
					.voxels
					.add_voxel(local, voxel);
			}
			None => {
				if let Some(slot) = self.subgrids.get_mut(&pos) {
					slot.voxels.remove_voxel(&local);
				}
			}
		}
		pos
	}

	pub fn clear_area(&mut self, min: IVec3, size: IVec3) -> HashSet<IVec3> {
		let mut touched = HashSet::new();
		if size.cmple(IVec3::ZERO).any() { return touched; }
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
		touched
	}

	pub fn read_clear_area(&mut self, min: IVec3, size: IVec3) -> (HashSet<IVec3>, Voxels) {
		let mut touched = HashSet::new();
		let mut out = Voxels::new();
		if size.cmple(IVec3::ZERO).any() { return (touched, out); }
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

					let region_lo = local.as_i16vec3();
					let region_hi = (local + extent).as_i16vec3();
					let palette = slot.voxels.palette();
					slot.voxels.grid_tree().for_each_in_region(
						region_lo,
						region_hi - I16Vec3::ONE,
						|pos, run, id| {
							let Some(voxel) = palette.voxel(id) else { return };
							let run_lo = pos.max(region_lo);
							let run_hi = (pos + I16Vec3::splat(run as i16)).min(region_hi);
							let out_extent = run_hi - run_lo;
							if out_extent.cmple(I16Vec3::ZERO).any() { return; }
							let world = sub_origin + run_lo.as_ivec3();
							out.add_area((world - min).as_i16vec3(), out_extent, *voxel);
						},
					);

					if local == IVec3::ZERO && extent == sub {
						slot.voxels = Voxels::new();
					} else {
						slot.voxels.remove_area(local.as_i16vec3(), extent.as_i16vec3());
					}
					touched.insert(sub_origin);
				}
			}
		}
		(touched, out)
	}

	pub fn read_area(&self, min: IVec3, size: IVec3) -> Voxels {
		let mut out = Voxels::new();
		if size.cmple(IVec3::ZERO).any() { return out; }
		let sub = IVec3::splat(SUB_GRID_SIZE);
		let hi = min + size;
		let sg_lo = min.div_euclid(sub);
		let sg_hi = (hi - IVec3::ONE).div_euclid(sub);
		for sx in sg_lo.x..=sg_hi.x {
			for sy in sg_lo.y..=sg_hi.y {
				for sz in sg_lo.z..=sg_hi.z {
					let sub_origin = IVec3::new(sx, sy, sz) * SUB_GRID_SIZE;
					let Some(slot) = self.subgrids.get(&sub_origin) else { continue };
					let cell_lo = min.max(sub_origin);
					let cell_hi = hi.min(sub_origin + sub);
					let local = cell_lo - sub_origin;
					let extent = cell_hi - cell_lo;

					let region_lo = local.as_i16vec3();
					let region_hi = (local + extent).as_i16vec3();
					let palette = slot.voxels.palette();
					slot.voxels.grid_tree().for_each_in_region(
						region_lo,
						region_hi - I16Vec3::ONE,
						|pos, run, id| {
							let Some(voxel) = palette.voxel(id) else { return };
							let run_lo = pos.max(region_lo);
							let run_hi = (pos + I16Vec3::splat(run as i16)).min(region_hi);
							let out_extent = run_hi - run_lo;
							if out_extent.cmple(I16Vec3::ZERO).any() { return; }
							let world = sub_origin + run_lo.as_ivec3();
							out.add_area((world - min).as_i16vec3(), out_extent, *voxel);
						},
					);
				}
			}
		}
		out
	}

	/// Write every voxel of `src` directly into this grid at offset `base`,
	/// bypassing the pending queue. Cells are bulk-filled and split across
	/// sub-grid boundaries. Returns the touched origins for [`reconcile_subgrids`].
	pub fn splat_voxels(&mut self, base: IVec3, src: &Voxels) -> HashSet<IVec3> {
		let _zone = span!();
		struct Batch {
			origin: IVec3,
			voxels: Vec<(I16Vec3, u16)>,
			areas: Vec<(I16Vec3, I16Vec3, u16)>,
			voxel_min: I16Vec3,
			voxel_max: I16Vec3,
		}
		impl Batch {
			fn new(origin: IVec3) -> Self {
				Self { origin, voxels: Vec::new(), areas: Vec::new(), voxel_min: I16Vec3::splat(i16::MAX), voxel_max: I16Vec3::splat(i16::MIN) }
			}
			fn push_voxel(&mut self, local: I16Vec3, palette_id: u16) {
				self.voxel_min = self.voxel_min.min(local);
				self.voxel_max = self.voxel_max.max(local);
				self.voxels.push((local, palette_id));
			}
		}
		let mut batches: Vec<Batch> = Vec::new();
		let palette = src.palette();
		let sub = IVec3::splat(SUB_GRID_SIZE);
		for (pos, size, palette_id) in src.grid_tree().iter() {
			if palette.voxel(palette_id).is_none() { continue; }
			let lo = base + pos.as_ivec3();
			if size == 1 {
				let sub_origin = lo.div_euclid(sub) * SUB_GRID_SIZE;
				let local = (lo - sub_origin).as_i16vec3();
				if let Some(batch) = batches.iter_mut().find(|batch| batch.origin == sub_origin) {
					batch.push_voxel(local, palette_id);
				} else {
					let mut batch = Batch::new(sub_origin);
					batch.push_voxel(local, palette_id);
					batches.push(batch);
				}
				continue;
			}
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
						if let Some(batch) = batches.iter_mut().find(|batch| batch.origin == sub_origin) {
							batch.areas.push((local, extent, palette_id));
						} else {
							let mut batch = Batch::new(sub_origin);
							batch.areas.push((local, extent, palette_id));
							batches.push(batch);
						}
					}
				}
			}
		}

		let mut touched = HashSet::with_capacity(batches.len());
		for batch in batches {
			let slot = self
				.subgrids
				.entry(batch.origin)
				.or_insert_with(|| SubGridSlot { voxels: Voxels::new(), entity: Entity::PLACEHOLDER });

			if !batch.voxels.is_empty() {
				slot.voxels.add_palette_voxels_in_bounds(&batch.voxels, palette, batch.voxel_min, batch.voxel_max);
			}
			if !batch.areas.is_empty() {
				slot.voxels.add_palette_areas(&batch.areas, palette);
			}
			touched.insert(batch.origin);
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

	/// Entities for non-empty sub-grids whose occupied voxel bounds intersect the
	/// half-open grid-local area `[min, min + size)`.
	///
	/// This does not expose or rely on the current fixed sub-grid extent; callers should
	/// use it when they need render/collision handles for a voxel area.
	pub fn subgrid_entities_in_area(&self, min: IVec3, size: IVec3) -> impl Iterator<Item = SubGridId> + '_ {
		let hi = min + size;
		self.subgrids.iter().filter_map(move |(sub_origin, slot)| {
			let (bounds_min, bounds_max) = slot.voxels.bounding_box()?;
			let occupied_min = *sub_origin + bounds_min.as_ivec3();
			let occupied_hi = *sub_origin + bounds_max.as_ivec3() + IVec3::ONE;
			(occupied_min.cmplt(hi).all() && occupied_hi.cmpgt(min).all()).then_some(slot.entity)
		})
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

pub fn reconcile_subgrids(
	grid_entity: Entity,
	grid: &mut Grid,
	touched: impl IntoIterator<Item = IVec3>,
	commands: &mut Commands,
	sub_grids: &mut Query<&mut SubGrid>,
) {
	let _zone = span!();
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
}
