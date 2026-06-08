use bevy::math::I16Vec3;
use tracy_client::span;
use std::{sync::{Mutex, atomic::{AtomicBool, Ordering}}};
use bimap::BiHashMap;

use super::voxel_grid_tree::VoxelGridTree;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Voxel {
	pub color: [u8; 4],
	pub mass: u32,
}

#[derive(Clone, Debug)]
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

#[derive(Debug)]
pub struct Voxels {
	voxels: VoxelGridTree,
	voxel_palette: VoxelPalette,
	bounding_box: Mutex<Option<(I16Vec3, I16Vec3)>>,
	bounding_box_dirty: AtomicBool,
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

	pub fn add_area(&mut self, pos: I16Vec3, size: I16Vec3, voxel: Voxel) {
		if size.cmple(I16Vec3::ZERO).any() { return; }
		let id = self.voxel_palette.palette_id(&voxel);
		self.voxels.add_area(&pos, size.as_ivec3(), id);
		let max = pos + size - I16Vec3::ONE;
		let bb = self.bounding_box.get_mut().unwrap();
		*bb = Some(match *bb {
			Some((mn, mx)) => (mn.min(pos), mx.max(max)),
			None => (pos, max),
		});
	}

	pub fn remove_voxel(&mut self, pos: &I16Vec3) -> Option<Voxel> {
		let out = self.voxel_palette.voxel(self.voxels.remove(pos)?).cloned();
		self.bounding_box_dirty.store(true, Ordering::Release);
		out
	}

	pub fn remove_area(&mut self, pos: I16Vec3, size: I16Vec3) {
		if size.cmple(I16Vec3::ZERO).any() { return; }
		self.voxels.remove_area(&pos, size.as_ivec3());
		self.bounding_box_dirty.store(true, Ordering::Release);
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
