use std::{collections::{HashMap, HashSet}, sync::{Mutex, atomic::{AtomicBool, Ordering}}};

use bevy::math::{I8Vec3, IVec2, IVec3, Quat, U16Vec3, Vec3};
use bevy::prelude::*;

use tracy_client::span;

use crate::bvh::BVH;
use crate::sdf::Sdf;
use crate::subgrid::{SubGrid, SubGridId, SubGridRef};
use crate::voxels::{Voxel, Voxels};

// Temporary default sub-grid extent. New sub-grids still start on this grid, but
// ownership is stored per sub-grid so future optimization can grow/shrink owned
// AABBs without changing the public sub-grid handle model.
const SUB_GRID_SIZE: i32 = 64;

/// A [`Grid`] entity. Each rigid voxel object owns one.
pub type GridId = Entity;

#[derive(Debug)]
pub(crate) struct SubGridSlot {
	pub(crate) voxels: Voxels,
	pub(crate) entity: SubGridId,
	owned_min: IVec3,
	owned_size: IVec3,
}

impl SubGridSlot {
	pub(crate) fn new_default(sub_grid_pos: IVec3) -> Self {
		Self { voxels: Voxels::new(), entity: Entity::PLACEHOLDER, owned_min: sub_grid_pos, owned_size: IVec3::splat(SUB_GRID_SIZE) }
	}

	fn owned_hi(&self) -> IVec3 {
		self.owned_min + self.owned_size
	}

	fn owns(&self, pos: IVec3) -> bool {
		pos.cmpge(self.owned_min).all() && pos.cmplt(self.owned_hi()).all()
	}

	fn owned_intersection(&self, min: IVec3, hi: IVec3) -> Option<(IVec3, IVec3)> {
		let cell_lo = min.max(self.owned_min);
		let cell_hi = hi.min(self.owned_hi());
		cell_lo.cmplt(cell_hi).all().then_some((cell_lo, cell_hi))
	}
}

#[derive(Debug, Component)]
pub struct Grid {
	pub(crate) subgrids: HashMap<IVec3, SubGridSlot>,
	subgrid_bvh: Mutex<Option<BVH<IVec3>>>,
	subgrid_bvh_dirty: AtomicBool,
}

impl Default for Grid {
	fn default() -> Self {
		Self { subgrids: HashMap::new(), subgrid_bvh: Mutex::new(None), subgrid_bvh_dirty: AtomicBool::new(false) }
	}
}

impl Grid {
	pub fn new() -> Self {
		Self::default()
	}

	fn default_sub_grid_pos_of(pos: IVec3) -> IVec3 {
		pos.div_euclid(IVec3::splat(SUB_GRID_SIZE)) * SUB_GRID_SIZE
	}

	fn sub_grid_pos_of(&self, pos: IVec3) -> IVec3 {
		let default = Self::default_sub_grid_pos_of(pos);
		if self.subgrids.get(&default).is_some_and(|slot| slot.owns(pos)) {
			return default;
		}
		self.subgrids.iter().find_map(|(sub_grid_pos, slot)| slot.owns(pos).then_some(*sub_grid_pos)).unwrap_or(default)
	}

	fn local_of(sub_grid_pos: IVec3, pos: IVec3) -> U16Vec3 {
		(pos - sub_grid_pos).as_u16vec3()
	}

	fn intersecting_subgrids(&self, min: IVec3, hi: IVec3) -> Vec<IVec3> {
		self.subgrids.iter().filter_map(|(sub_grid_pos, slot)| slot.owned_intersection(min, hi).is_some().then_some(*sub_grid_pos)).collect()
	}

	pub(crate) fn write_regions(&self, min: IVec3, hi: IVec3) -> Vec<(IVec3, IVec3, IVec3)> {
		if min.cmpge(hi).any() {
			return Vec::new();
		}

		fn add_default_cuts(cuts: &mut Vec<i32>, lo: i32, hi: i32) {
			cuts.push(lo);
			cuts.push(hi);
			let size = SUB_GRID_SIZE;
			let mut boundary = lo.div_euclid(size) * size;
			while boundary <= hi {
				if boundary > lo && boundary < hi {
					cuts.push(boundary);
				}
				boundary += size;
			}
		}

		let mut x_cuts = Vec::new();
		let mut y_cuts = Vec::new();
		let mut z_cuts = Vec::new();
		add_default_cuts(&mut x_cuts, min.x, hi.x);
		add_default_cuts(&mut y_cuts, min.y, hi.y);
		add_default_cuts(&mut z_cuts, min.z, hi.z);

		for slot in self.subgrids.values() {
			let Some((lo, region_hi)) = slot.owned_intersection(min, hi) else { continue };
			x_cuts.extend([lo.x, region_hi.x]);
			y_cuts.extend([lo.y, region_hi.y]);
			z_cuts.extend([lo.z, region_hi.z]);
		}

		x_cuts.sort_unstable();
		y_cuts.sort_unstable();
		z_cuts.sort_unstable();
		x_cuts.dedup();
		y_cuts.dedup();
		z_cuts.dedup();

		let mut regions = Vec::new();
		for x in x_cuts.windows(2) {
			for y in y_cuts.windows(2) {
				for z in z_cuts.windows(2) {
					let cell_lo = IVec3::new(x[0], y[0], z[0]);
					let cell_hi = IVec3::new(x[1], y[1], z[1]);
					regions.push((self.sub_grid_pos_of(cell_lo), cell_lo, cell_hi));
				}
			}
		}
		regions
	}

	pub(crate) fn mark_subgrid_bvh_dirty(&mut self) {
		self.subgrid_bvh_dirty.store(true, Ordering::Release);
	}

	fn occupied_subgrid_bounds(sub_grid_pos: IVec3, slot: &SubGridSlot) -> Option<(IVec3, IVec3)> {
		let (bounds_min, bounds_max) = slot.voxels.bounding_box()?;
		Some((sub_grid_pos + bounds_min.as_ivec3(), sub_grid_pos + bounds_max.as_ivec3() + IVec3::ONE))
	}

	fn rebuild_subgrid_bvh(&self) {
		let items = self.subgrids.iter().filter_map(|(sub_grid_pos, slot)| {
			let (min, hi) = Self::occupied_subgrid_bounds(*sub_grid_pos, slot)?;
			Some((*sub_grid_pos, (min.as_vec3(), hi.as_vec3())))
		}).collect();
		*self.subgrid_bvh.lock().unwrap() = Some(BVH::new(items));
		self.subgrid_bvh_dirty.store(false, Ordering::Release);
	}

	fn ensure_subgrid_bvh(&self) {
		if self.subgrid_bvh_dirty.load(Ordering::Acquire) {
			self.rebuild_subgrid_bvh();
			return;
		}
		let needs_init = {
			let guard = self.subgrid_bvh.lock().unwrap();
			guard.is_none() && !self.subgrids.is_empty()
		};
		if needs_init {
			self.rebuild_subgrid_bvh();
		}
	}

	fn subgrid_origins_in_area(&self, min: IVec3, size: IVec3) -> Vec<IVec3> {
		if size.cmple(IVec3::ZERO).any() || self.subgrids.is_empty() {
			return Vec::new();
		}
		let hi = min + size;
		self.ensure_subgrid_bvh();
		let candidates = {
			let guard = self.subgrid_bvh.lock().unwrap();
			guard.as_ref().map(|bvh| bvh.collisions(&(min.as_vec3(), hi.as_vec3()))).unwrap_or_default()
		};
		candidates.into_iter().filter(|sub_grid_pos| {
			let Some(slot) = self.subgrids.get(sub_grid_pos) else { return false };
			let Some((occupied_min, occupied_hi)) = Self::occupied_subgrid_bounds(*sub_grid_pos, slot) else { return false };
			occupied_min.cmplt(hi).all() && occupied_hi.cmpgt(min).all()
		}).collect()
	}

	fn read_region_into(out: &mut Voxels, out_min: IVec3, sub_grid_pos: IVec3, slot: &SubGridSlot, cell_lo: IVec3, cell_hi: IVec3) {
		let _span = span!();
		let region_lo = Self::local_of(sub_grid_pos, cell_lo);
		let region_hi = Self::local_of(sub_grid_pos, cell_hi);
		let mut areas = Vec::new();
		slot.voxels.grid_tree().for_each_in_region(crate::grid_tree::GridRegion::from_min_size(region_lo.as_ivec3(), (region_hi - region_lo).as_ivec3()).unwrap(), |pos, run, id| {
			let run_lo = pos.max(region_lo);
			let run_hi = (pos + U16Vec3::splat(run as u16)).min(region_hi);
			let out_extent = run_hi - run_lo;
			if out_extent == U16Vec3::ZERO { return; }
			let world = sub_grid_pos + run_lo.as_ivec3();
			areas.push(((world - out_min).as_u16vec3(), out_extent, id));
		});
		out.add_palette_areas(&areas, slot.voxels.palette());
	}

	/// Write (`Some`) or remove (`None`) a single voxel. Returns the touched
	/// sub-grid position for [`reconcile_subgrids`].
	pub fn set_voxel(&mut self, voxel_pos: IVec3, voxel: Option<Voxel>) -> IVec3 {
		let sub_grid_pos = self.sub_grid_pos_of(voxel_pos);
		let local = Self::local_of(sub_grid_pos, voxel_pos);
		match voxel {
			Some(voxel) => {
				self.subgrids.entry(sub_grid_pos).or_insert_with(|| SubGridSlot::new_default(sub_grid_pos)).voxels.add_voxel(local, voxel);
			}
			None => {
				if let Some(slot) = self.subgrids.get_mut(&sub_grid_pos) {
					slot.voxels.remove_voxel(&local);
				}
			}
		}
		self.mark_subgrid_bvh_dirty();
		sub_grid_pos
	}

	pub fn clear_area(&mut self, min: IVec3, size: IVec3) -> HashSet<IVec3> {
		let mut touched = HashSet::new();
		if size.cmple(IVec3::ZERO).any() {
			return touched;
		}
		let hi = min + size;
		for sub_grid_pos in self.intersecting_subgrids(min, hi) {
			let Some(slot) = self.subgrids.get_mut(&sub_grid_pos) else { continue };
			let Some((cell_lo, cell_hi)) = slot.owned_intersection(min, hi) else { continue };
			let local = Self::local_of(sub_grid_pos, cell_lo);
			let extent = (cell_hi - cell_lo).as_u16vec3();
			if cell_lo == slot.owned_min && cell_hi == slot.owned_hi() {
				slot.voxels = Voxels::new();
			} else {
				slot.voxels.remove_area(local, extent);
			}
			touched.insert(sub_grid_pos);
		}
		if !touched.is_empty() {
			self.mark_subgrid_bvh_dirty();
		}
		touched
	}

	pub fn read_clear_area(&mut self, min: IVec3, size: IVec3) -> (HashSet<IVec3>, Voxels) {
		let mut touched = HashSet::new();
		let mut out = Voxels::new();
		if size.cmple(IVec3::ZERO).any() {
			return (touched, out);
		}
		let hi = min + size;
		for sub_grid_pos in self.intersecting_subgrids(min, hi) {
			let Some(slot) = self.subgrids.get_mut(&sub_grid_pos) else { continue };
			let Some((cell_lo, cell_hi)) = slot.owned_intersection(min, hi) else { continue };
			Self::read_region_into(&mut out, min, sub_grid_pos, slot, cell_lo, cell_hi);

			let local = Self::local_of(sub_grid_pos, cell_lo);
			let extent = (cell_hi - cell_lo).as_u16vec3();
			if cell_lo == slot.owned_min && cell_hi == slot.owned_hi() {
				slot.voxels = Voxels::new();
			} else {
				slot.voxels.remove_area(local, extent);
			}
			touched.insert(sub_grid_pos);
		}
		if !touched.is_empty() {
			self.mark_subgrid_bvh_dirty();
		}
		(touched, out)
	}

	pub fn read_area(&self, min: IVec3, size: IVec3) -> Voxels {
		let _span = span!();
		let mut out = Voxels::new();
		if size.cmple(IVec3::ZERO).any() {
			return out;
		}
		let hi = min + size;
		for (sub_grid_pos, slot) in &self.subgrids {
			let Some((cell_lo, cell_hi)) = slot.owned_intersection(min, hi) else { continue };
			Self::read_region_into(&mut out, min, *sub_grid_pos, slot, cell_lo, cell_hi);
		}
		out
	}

	/// Write every voxel of `src` directly into this grid at offset `base`,
	/// bypassing the pending queue. Cells are bulk-filled and split across
	/// sub-grid ownership regions. Returns the touched positions for
	/// [`reconcile_subgrids`].
	pub fn splat_voxels(&mut self, base: IVec3, src: &Voxels) -> HashSet<IVec3> {
		let _zone = span!();
		let mut touched = HashSet::new();
		let Some((src_min, src_max)) = src.bounding_box() else { return touched };
		let world_min = base + src_min.as_ivec3();
		let world_end = base + src_max.as_ivec3() + IVec3::ONE;

		for (sub_grid_pos, cell_lo, cell_hi) in self.write_regions(world_min, world_end) {
			let source_min = cell_lo - base;
			let source_size = cell_hi - cell_lo;
			let Some(source_region) = crate::grid_tree::GridRegion::from_min_size(source_min, source_size) else { continue };
			let slot = self.subgrids.entry(sub_grid_pos).or_insert_with(|| SubGridSlot::new_default(sub_grid_pos));
			slot.voxels.merge_region_from(src, Some(source_region), base - sub_grid_pos);
			touched.insert(sub_grid_pos);
		}
		if !touched.is_empty() {
			self.mark_subgrid_bvh_dirty();
		}
		touched
	}

	pub fn apply_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize, voxel: Voxel) -> HashSet<IVec3> {
		let mut touched = HashSet::new();
		let min = initial_min.floor().as_ivec3();
		let hi = initial_max.ceil().as_ivec3();
		for (sub_grid_pos, cell_lo, cell_hi) in self.write_regions(min, hi) {
			let local_min = (cell_lo - sub_grid_pos).as_vec3();
			let local_max = (cell_hi - sub_grid_pos).as_vec3();
			let local_sdf = |p: Vec3| sdf.sample(p + sub_grid_pos.as_vec3());
			let slot = self.subgrids.entry(sub_grid_pos).or_insert_with(|| SubGridSlot::new_default(sub_grid_pos));
			slot.voxels.apply_sdf(local_min, local_max, &local_sdf, face_resolution, iterations, voxel);
			touched.insert(sub_grid_pos);
		}
		if !touched.is_empty() {
			self.mark_subgrid_bvh_dirty();
		}
		touched
	}

	pub fn clear_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize) -> HashSet<IVec3> {
		let mut touched = HashSet::new();
		let min = initial_min.floor().as_ivec3();
		let hi = initial_max.ceil().as_ivec3();
		for (sub_grid_pos, cell_lo, cell_hi) in self.write_regions(min, hi) {
			let Some(slot) = self.subgrids.get_mut(&sub_grid_pos) else { continue };
			let local_min = (cell_lo - sub_grid_pos).as_vec3();
			let local_max = (cell_hi - sub_grid_pos).as_vec3();
			let local_sdf = |p: Vec3| sdf.sample(p + sub_grid_pos.as_vec3());
			slot.voxels.clear_sdf(local_min, local_max, &local_sdf, face_resolution, iterations);
			touched.insert(sub_grid_pos);
		}
		if !touched.is_empty() {
			self.mark_subgrid_bvh_dirty();
		}
		touched
	}

	pub fn voxel(&self, voxel_pos: &IVec3) -> Option<&Voxel> {
		let sub_grid_pos = self.sub_grid_pos_of(*voxel_pos);
		self.subgrids.get(&sub_grid_pos)?.voxels.voxel(&Self::local_of(sub_grid_pos, *voxel_pos))
	}

	pub fn view(&self, sub_grid: &SubGrid) -> Option<SubGridRef<'_>> {
		let slot = self.subgrids.get(&sub_grid.sub_grid_pos())?;
		Some(SubGridRef::new(&slot.voxels, sub_grid.sub_grid_pos()))
	}

	pub fn subgrids(&self) -> impl Iterator<Item = SubGridRef<'_>> {
		self.subgrids.iter().map(|(pos, slot)| SubGridRef::new(&slot.voxels, *pos))
	}

	/// True when the sub-grid's owned AABB intersects the half-open grid-local area
	/// `[min, min + size)`. Intended for callers that are already handling touched
	/// sub-grid positions returned by grid mutation methods.
	pub fn subgrid_owned_area_intersects(&self, sub_grid_pos: IVec3, min: IVec3, size: IVec3) -> bool {
		if size.cmple(IVec3::ZERO).any() {
			return false;
		}
		let Some(slot) = self.subgrids.get(&sub_grid_pos) else { return false };
		slot.owned_intersection(min, min + size).is_some()
	}

	/// Entities for non-empty sub-grids whose occupied voxel bounds intersect the
	/// half-open grid-local area `[min, min + size)`.
	pub fn subgrid_entities_in_area(&self, min: IVec3, size: IVec3) -> impl Iterator<Item = SubGridId> + '_ {
		self.subgrid_origins_in_area(min, size)
			.into_iter()
			.filter_map(|sub_grid_pos| self.subgrids.get(&sub_grid_pos).map(|slot| slot.entity))
	}

	pub fn raycast(&self, origin: Vec3, dir: Vec3) -> Option<GridRaycastHit> {
		if self.subgrids.is_empty() {
			return None;
		}
		self.ensure_subgrid_bvh();
		let transform = Transform { translation: origin, rotation: Quat::from_rotation_arc(Vec3::Z, dir), scale: Vec3::ONE };
		let candidates = {
			let guard = self.subgrid_bvh.lock().unwrap();
			guard.as_ref().map(|bvh| bvh.raycast(&transform, None).collect::<Vec<_>>()).unwrap_or_default()
		};
		let mut best: Option<GridRaycastHit> = None;
		for (sub_grid_pos, entry_distance) in candidates {
			if best.as_ref().is_some_and(|hit| entry_distance > hit.distance) {
				break;
			}
			let Some(slot) = self.subgrids.get(&sub_grid_pos) else { continue };
			let sub_grid = SubGridRef::new(&slot.voxels, sub_grid_pos);
			if let Some(hit) = sub_grid.raycast(origin - sub_grid_pos.as_vec3(), dir, best.as_ref().map(|current| current.distance)) {
				best = Some(GridRaycastHit {
					voxel_pos: sub_grid_pos + hit.voxel_pos.as_ivec3(),
					normal: hit.normal.as_ivec3(),
					distance: hit.distance,
				});
			}
		}
		best
	}
}

pub struct GridRaycastHit {
	pub voxel_pos: IVec3,
	pub normal: IVec3,
	pub distance: f32,
}

pub struct SubGridRaycastHit {
	pub voxel_pos: U16Vec3,
	pub normal: I8Vec3,
	pub distance: f32,
}

impl SubGridRef<'_> {
	pub fn raycast(&self, origin: Vec3, dir: Vec3, max_distance: Option<f32>) -> Option<SubGridRaycastHit> {
		// GridTree::raycast takes a Transform whose rotation maps +Z to the ray dir.
		let transform = Transform { translation: origin, rotation: Quat::from_rotation_arc(Vec3::Z, dir), scale: Vec3::ONE };
		self.voxels().grid_tree().raycast(&transform, max_distance).map(|(voxel_pos, normal, distance)| SubGridRaycastHit { voxel_pos, normal, distance })
	}
}

pub fn reconcile_subgrids(
	grid_entity: Entity, grid: &mut Grid, touched: impl IntoIterator<Item = IVec3>, commands: &mut Commands, sub_grids: &mut Query<&mut SubGrid>,
) {
	let _zone = span!();
	for pos in touched {
		let Some(slot) = grid.subgrids.get(&pos) else { continue };
		let entity = slot.entity;
		if slot.voxels.is_empty() {
			if entity != Entity::PLACEHOLDER {
				commands.entity(entity).despawn();
			}
			grid.subgrids.remove(&pos);
			grid.mark_subgrid_bvh_dirty();
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
	use bevy::math::U16Vec3;

	fn vox(c: u8) -> Voxel {
		Voxel { color: [c, c, c, 255], mass: 1 }
	}

	#[test]
	fn splat_voxels_blocking_uses_tree_merge_for_empty_single_subgrid() {
		bevy::tasks::ComputeTaskPool::get_or_init(|| bevy::tasks::TaskPoolBuilder::new().build());
		let mut src = Voxels::new();
		src.add_area(U16Vec3::ZERO, U16Vec3::splat(16), vox(7));

		let mut grid = Grid::new();
		let touched = crate::splat::splat_voxels_blocking(std::slice::from_mut(&mut grid), &[crate::splat::GridSplat { grid: 0, base: IVec3::ZERO, voxels: &src }]);

		assert_eq!(touched.get(&0).cloned().unwrap_or_default(), HashSet::from([IVec3::ZERO]));
		let slot = grid.subgrids.get(&IVec3::ZERO).unwrap();
		assert_eq!(slot.voxels.grid_tree().iter().collect::<Vec<_>>(), src.grid_tree().iter().collect::<Vec<_>>());
	}

	#[test]
	fn splat_voxels_uses_tree_merge_for_empty_single_subgrid() {
		let mut src = Voxels::new();
		src.add_area(U16Vec3::ZERO, U16Vec3::splat(16), vox(7));

		let mut grid = Grid::new();
		let touched = grid.splat_voxels(IVec3::ZERO, &src);

		assert_eq!(touched, HashSet::from([IVec3::ZERO]));
		let slot = grid.subgrids.get(&IVec3::ZERO).unwrap();
		assert_eq!(slot.voxels.grid_tree().iter().collect::<Vec<_>>(), src.grid_tree().iter().collect::<Vec<_>>());
	}

	#[test]
	fn splat_voxels_reproduces_source() {
		// Negative, non-aligned base so cells straddle sub-grid boundaries.
		let mut src = Voxels::new();
		for x in 0..40 {
			for y in 0..40 {
				for z in 0..40 {
					src.add_voxel(U16Vec3::new(x as u16, y as u16, z as u16), vox(1));
				}
			}
		}
		for &(x, y, z, c) in &[(5i16, 60i16, 3i16, 2u8), (70, 1, 70, 3), (33, 33, 33, 4)] {
			src.add_voxel(U16Vec3::new(x as u16, y as u16, z as u16), vox(c));
		}

		let base = IVec3::new(-17, 5, -70);
		let mut grid = Grid::new();
		grid.splat_voxels(base, &src);

		let mut count = 0u64;
		for (pos, size, id) in src.grid_tree().iter() {
			let voxel = *src.palette().voxel(id).unwrap();
			for dx in 0..size as i32 {
				for dy in 0..size as i32 {
					for dz in 0..size as i32 {
						let p = base + pos.as_ivec3() + IVec3::new(dx, dy, dz);
						assert_eq!(grid.voxel(&p), Some(&voxel), "mismatch at {p:?}");
						count += 1;
					}
				}
			}
		}
		let total: u64 = grid.subgrids.values().map(|s| s.voxels.grid_tree().len()).sum();
		assert_eq!(total, count);
	}

	#[test]
	fn apply_sdf_places_single_voxel_from_exact_bounds() {
		let mut grid = Grid::new();
		let sdf = |q: Vec3| if (q - Vec3::new(16.5, 0.5, 0.5)).length() < 0.1 { -1.0 } else { 1.0 };
		grid.apply_sdf(Vec3::new(16.0, 0.0, 0.0), Vec3::new(17.0, 1.0, 1.0), &sdf, IVec2::splat(3), 2, vox(9));
		assert_eq!(grid.voxel(&IVec3::new(16, 0, 0)), Some(&vox(9)));
	}

	#[test]
	fn subgrid_area_queries_use_occupied_bounds() {
		let mut grid = Grid::new();
		grid.set_voxel(IVec3::new(1, 1, 1), Some(vox(1)));
		grid.set_voxel(IVec3::new(65, 1, 1), Some(vox(2)));

		assert_eq!(grid.subgrid_origins_in_area(IVec3::ZERO, IVec3::splat(64)), vec![IVec3::ZERO]);
		assert_eq!(grid.subgrid_origins_in_area(IVec3::new(64, 0, 0), IVec3::splat(64)), vec![IVec3::new(64, 0, 0)]);
		assert!(grid.subgrid_origins_in_area(IVec3::new(128, 0, 0), IVec3::splat(64)).is_empty());
	}

	#[test]
	fn subgrid_bvh_stays_in_sync_after_blocking_splat() {
		bevy::tasks::ComputeTaskPool::get_or_init(|| bevy::tasks::TaskPoolBuilder::new().build());
		let mut src = Voxels::new();
		src.add_area(U16Vec3::ZERO, U16Vec3::splat(8), vox(7));
		let mut grid = Grid::new();

		crate::splat::splat_voxels_blocking(std::slice::from_mut(&mut grid), &[crate::splat::GridSplat { grid: 0, base: IVec3::new(64, 0, 0), voxels: &src }]);

		assert_eq!(grid.subgrid_origins_in_area(IVec3::new(64, 0, 0), IVec3::splat(64)), vec![IVec3::new(64, 0, 0)]);
	}
}
