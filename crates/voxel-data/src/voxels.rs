use bevy::math::{I16Vec3, IVec2, Vec3};
use serde::{Deserialize, Serialize};
use tracy_client::span;
use std::{collections::HashMap, sync::{Mutex, atomic::{AtomicBool, Ordering}}};
use bimap::BiHashMap;

use super::{grid_tree::{GridRegion, size as grid_tree_size}, sdf::Sdf, voxel_grid_tree::VoxelGridTree};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Voxel {
	pub color: [u8; 4],
	pub mass: u32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VoxelPalette {
	pub palette: BiHashMap<u16, Voxel>,
	next_id: u16,
}

impl VoxelPalette {
	pub fn new() -> Self {
		Self {
			palette: BiHashMap::new(),
			next_id: 0,
		}
	}
	pub fn palette_id(&mut self, voxel: &Voxel) -> u16 {
		if self.palette.insert_no_overwrite(self.next_id, *voxel).is_err() {
			let id = *self.palette.get_by_right(voxel).unwrap();
			assert!(self.next_id != id);
			return id;
		}
		let id = self.next_id;
		self.next_id += 1;
		id
	}
	pub fn voxel(&self, id: u16) -> Option<&Voxel> {
		self.palette.get_by_left(&id)
	}
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Voxels {
	voxels: VoxelGridTree,
	voxel_palette: VoxelPalette,
	#[serde(skip, default = "default_bounding_box")]
	bounding_box: Mutex<Option<(I16Vec3, I16Vec3)>>,
	#[serde(skip, default = "default_bounding_box_dirty")]
	bounding_box_dirty: AtomicBool,
}

fn default_bounding_box() -> Mutex<Option<(I16Vec3, I16Vec3)>> {
	Mutex::new(None)
}

fn default_bounding_box_dirty() -> AtomicBool {
	AtomicBool::new(true)
}

impl Clone for Voxels {
	fn clone(&self) -> Self {
		Self {
			voxels: self.voxels.clone(),
			voxel_palette: self.voxel_palette.clone(),
			bounding_box: Mutex::new(*self.bounding_box.lock().unwrap()),
			bounding_box_dirty: AtomicBool::new(self.bounding_box_dirty.load(Ordering::Relaxed)),
		}
	}
}

impl Voxels {
	pub fn new() -> Self {
		Self {
			voxels: VoxelGridTree::new(),
			voxel_palette: VoxelPalette::new(),
			bounding_box: Mutex::new(None),
			bounding_box_dirty: AtomicBool::new(false)
		}
	}
	pub fn add_voxel(&mut self, pos: I16Vec3, voxel: Voxel) -> Option<Voxel> {
		let bounding_box = self.bounding_box.lock().unwrap().clone();
		*self.bounding_box.get_mut().unwrap() = Some(if let Some(bounding_box) = bounding_box {
			(bounding_box.0.min(pos), bounding_box.1.max(pos))
		} else {
			(pos, pos)
		});
		let out = self.voxels.insert(&pos, self.voxel_palette.palette_id(&voxel))?;
		self.voxel_palette.voxel(out).cloned()
	}

	pub fn ensure_area_covered(&mut self, pos: I16Vec3, size: I16Vec3) -> bool {
		self.voxels.ensure_area_covered(&pos, size.as_ivec3())
	}

	pub fn add_area(&mut self, pos: I16Vec3, size: I16Vec3, voxel: Voxel) {
		self.add_areas(&[(pos, size, voxel)]);
	}

	pub fn add_voxels(&mut self, voxels: &[(I16Vec3, Voxel)]) {
		if voxels.is_empty() {
			return;
		}
		let mut palette_cache = HashMap::new();
		let mut tree_voxels = Vec::with_capacity(voxels.len());
		let mut bounds: Option<(I16Vec3, I16Vec3)> = None;
		for (pos, voxel) in voxels {
			let id = *palette_cache.entry(*voxel).or_insert_with(|| self.voxel_palette.palette_id(voxel));
			tree_voxels.push((*pos, id));
			bounds = Some(match bounds {
				Some((mn, mx)) => (mn.min(*pos), mx.max(*pos)),
				None => (*pos, *pos),
			});
		}
		let Some((min, max)) = bounds else { return };
		self.voxels.add_single_voxels_in_bounds(&tree_voxels, min.as_ivec3(), max.as_ivec3());
		let bb = self.bounding_box.get_mut().unwrap();
		*bb = Some(match *bb {
			Some((mn, mx)) => (mn.min(min), mx.max(max)),
			None => (min, max),
		});
	}

	pub fn add_areas(&mut self, areas: &[(I16Vec3, I16Vec3, Voxel)]) {
		let mut tree_areas = Vec::with_capacity(areas.len());
		let mut palette_cache = HashMap::new();
		let mut bounds: Option<(I16Vec3, I16Vec3)> = None;
		for (pos, size, voxel) in areas {
			if size.cmple(I16Vec3::ZERO).any() { continue; }
			let id = *palette_cache.entry(*voxel).or_insert_with(|| self.voxel_palette.palette_id(voxel));
			tree_areas.push((*pos, size.as_ivec3(), id));
			let max = *pos + *size - I16Vec3::ONE;
			bounds = Some(match bounds {
				Some((mn, mx)) => (mn.min(*pos), mx.max(max)),
				None => (*pos, max),
			});
		}
		self.add_tree_areas(tree_areas, bounds);
	}

	pub fn add_palette_voxels(&mut self, voxels: &[(I16Vec3, u16)], source_palette: &VoxelPalette) {
		if voxels.is_empty() { return; }
		let (min, max) = voxels.iter().fold((I16Vec3::splat(i16::MAX), I16Vec3::splat(i16::MIN)), |(mn, mx), (pos, _)| (mn.min(*pos), mx.max(*pos)));
		self.add_palette_voxels_in_bounds(voxels, source_palette, min, max);
	}

	pub fn add_palette_voxels_in_bounds(&mut self, voxels: &[(I16Vec3, u16)], source_palette: &VoxelPalette, min: I16Vec3, max: I16Vec3) {
		if voxels.is_empty() { return; }
		if self.is_empty() {
			self.voxel_palette = source_palette.clone();
			self.voxels.add_single_voxels_in_bounds(voxels, min.as_ivec3(), max.as_ivec3());
		} else {
			let mut palette_cache = HashMap::new();
			let mapped: Vec<_> = voxels.iter().map(|(pos, source_id)| {
				let id = *palette_cache.entry(*source_id).or_insert_with(|| {
					let voxel = source_palette.voxel(*source_id).expect("source palette id missing");
					self.voxel_palette.palette_id(voxel)
				});
				(*pos, id)
			}).collect();
			self.voxels.add_single_voxels_in_bounds(&mapped, min.as_ivec3(), max.as_ivec3());
		}
		let bb = self.bounding_box.get_mut().unwrap();
		*bb = Some(match *bb {
			Some((mn, mx)) => (mn.min(min), mx.max(max)),
			None => (min, max),
		});
	}

	pub fn add_palette_areas(&mut self, areas: &[(I16Vec3, I16Vec3, u16)], source_palette: &VoxelPalette) {
		let mut tree_areas = Vec::with_capacity(areas.len());
		let mut bounds: Option<(I16Vec3, I16Vec3)> = None;
		if self.is_empty() {
			self.voxel_palette = source_palette.clone();
			for (pos, size, id) in areas {
				if size.cmple(I16Vec3::ZERO).any() { continue; }
				tree_areas.push((*pos, size.as_ivec3(), *id));
				let max = *pos + *size - I16Vec3::ONE;
				bounds = Some(match bounds {
					Some((mn, mx)) => (mn.min(*pos), mx.max(max)),
					None => (*pos, max),
				});
			}
		} else {
			let mut palette_cache = HashMap::new();
			for (pos, size, source_id) in areas {
				if size.cmple(I16Vec3::ZERO).any() { continue; }
				let id = *palette_cache.entry(*source_id).or_insert_with(|| {
					let voxel = source_palette.voxel(*source_id).expect("source palette id missing");
					self.voxel_palette.palette_id(voxel)
				});
				tree_areas.push((*pos, size.as_ivec3(), id));
				let max = *pos + *size - I16Vec3::ONE;
				bounds = Some(match bounds {
					Some((mn, mx)) => (mn.min(*pos), mx.max(max)),
					None => (*pos, max),
				});
			}
		}
		self.add_tree_areas(tree_areas, bounds);
	}

	fn add_tree_areas(&mut self, tree_areas: Vec<(I16Vec3, bevy::math::IVec3, u16)>, bounds: Option<(I16Vec3, I16Vec3)>) {
		if tree_areas.is_empty() {
			return;
		}
		self.voxels.add_areas(&tree_areas);
		let Some((min, max)) = bounds else { return };
		let bb = self.bounding_box.get_mut().unwrap();
		*bb = Some(match *bb {
			Some((mn, mx)) => (mn.min(min), mx.max(max)),
			None => (min, max),
		});
	}

	pub fn remove_voxel(&mut self, pos: &I16Vec3) -> Option<Voxel> {
		let out = self.voxel_palette.voxel(self.voxels.remove(pos)?).cloned();
		self.bounding_box_dirty.store(true, Ordering::Release);
		out
	}

	pub fn remove_area(&mut self, pos: I16Vec3, size: I16Vec3) {
		let Some(region) = GridRegion::from_min_size(pos.as_ivec3(), size.as_ivec3()) else { return };
		self.voxels.clear_region(region);
		self.bounding_box_dirty.store(true, Ordering::Release);
	}

	pub fn apply_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize, voxel: Voxel) {
		let id = self.voxel_palette.palette_id(&voxel);
		self.voxels.apply_sdf(initial_min, initial_max, sdf, face_resolution, iterations, id);
		self.bounding_box_dirty.store(true, Ordering::Release);
	}

	pub fn clear_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize) {
		self.voxels.clear_sdf(initial_min, initial_max, sdf, face_resolution, iterations);
		self.bounding_box_dirty.store(true, Ordering::Release);
	}

	pub fn merge_from(&mut self, source: &Voxels, offset: bevy::math::IVec3) {
		self.merge_region_from(source, None, offset);
	}

	pub fn merge_region_from(&mut self, source: &Voxels, source_region: Option<GridRegion>, offset: bevy::math::IVec3) {
		let mut bounds: Option<(I16Vec3, I16Vec3)> = None;
		let mut include_bounds = |min: bevy::math::IVec3, end: bevy::math::IVec3| {
			let bb_min = (min + offset).as_i16vec3();
			let bb_max = (end + offset - bevy::math::IVec3::ONE).as_i16vec3();
			bounds = Some(match bounds {
				Some((lo, hi)) => (lo.min(bb_min), hi.max(bb_max)),
				None => (bb_min, bb_max),
			});
		};

		match source_region {
			Some(region) => source.voxels.for_each_in_region(region, |pos, size, _| {
				let run_min = pos.as_ivec3().max(region.min);
				let run_end = (pos.as_ivec3() + bevy::math::IVec3::splat(size as i32)).min(region.end);
				if run_min.cmplt(run_end).all() {
					include_bounds(run_min, run_end);
				}
			}),
			None => {
				for (pos, size, _) in source.voxels.iter() {
					let min = pos.as_ivec3();
					include_bounds(min, min + bevy::math::IVec3::splat(size as i32));
				}
			}
		}

		if self.voxels.is_empty() {
			self.voxel_palette = source.voxel_palette.clone();
			match source_region {
				Some(region) => self.voxels.merge_region_from(&source.voxels, region, offset),
				None => self.voxels.merge_tree(&source.voxels, offset),
			}
		} else {
			let mut palette_cache = HashMap::new();
			let palette = &mut self.voxel_palette;
			let mut map_id = |source_id| {
				*palette_cache.entry(source_id).or_insert_with(|| {
					let voxel = source.voxel_palette.voxel(source_id).expect("source palette id missing");
					palette.palette_id(voxel)
				})
			};
			match source_region {
				Some(region) => self.voxels.merge_region_from_mapped(&source.voxels, region, offset, &mut map_id),
				None => {
					let (_, root_pos, root_depth) = source.voxels.internals();
					let root = GridRegion { min: root_pos.as_ivec3(), end: root_pos.as_ivec3() + bevy::math::IVec3::splat(grid_tree_size(root_depth) as i32) };
					self.voxels.merge_region_from_mapped(&source.voxels, root, offset, &mut map_id);
				}
			}
		}

		if let Some((min, max)) = bounds {
			let bb = self.bounding_box.get_mut().unwrap();
			*bb = Some(match *bb {
				Some((lo, hi)) => (lo.min(min), hi.max(max)),
				None => (min, max),
			});
		}
	}

	pub fn voxel(&self, pos: &I16Vec3) -> Option<&Voxel> {
			self.voxel_palette.voxel(self.voxels.get(pos)?)
		}
	pub fn grid_tree(&self) -> &VoxelGridTree { &self.voxels }
	pub fn palette(&self) -> &VoxelPalette { &self.voxel_palette }

	pub fn is_empty(&self) -> bool { self.voxels.len() == 0 }

	pub fn bounding_box(&self) -> Option<(I16Vec3, I16Vec3)> {
		if self.bounding_box_dirty.load(Ordering::Acquire) {
			let _zone = span!("rebuild voxel bounding box");
			self.bounding_box_dirty.store(false, Ordering::Release);
			*(self.bounding_box.lock()).unwrap() = self.voxels.iter().fold(None, |bb, (p, size, _)| {
				match bb {
					Some((min, max)) => Some((min.min(p), max.max(p + size as i16 - 1))),
					None => Some((p, p + size as i16 - 1))
				}
			});
		}
		self.bounding_box.lock().unwrap().clone()
	}
}
