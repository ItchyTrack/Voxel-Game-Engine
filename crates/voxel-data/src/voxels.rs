use glam::I16Vec3;
use tracy_client::span;
use std::cell::Cell;
use bimap::BiHashMap;

use super::grid_tree::GridTree;

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
	pub fn get_palette_id(&mut self, voxel: &Voxel) -> u16 {
		if let Err(_) = self.palette.insert_no_overwrite(self.next_id, *voxel) {
			let id = *self.palette.get_by_right(voxel).unwrap();
			assert!(self.next_id != id);
			return id;
		}
		let id = self.next_id;
		self.next_id += 1;
		id
	}
	pub fn get_voxel(&self, id: u16) -> Option<&Voxel> {
		self.palette.get_by_left(&id)
	}
}

pub struct Voxels {
	voxels: GridTree,
	voxel_palette: VoxelPalette,
	bounding_box: Cell<Option<(I16Vec3, I16Vec3)>>,
	bounding_box_dirty: Cell<bool>,
}

impl Voxels {
	pub fn new() -> Self {
		Self {
			voxels: GridTree::new(),
			voxel_palette: VoxelPalette::new(),
			bounding_box: Cell::new(None),
			bounding_box_dirty: Cell::new(false)
		}
	}
	pub fn add_voxel(&mut self, pos: I16Vec3, voxel: Voxel) -> Option<Voxel> {
		match self.bounding_box.get() {
			Some((min, max)) => {
				self.bounding_box.set(Some((min.min(pos), max.max(pos))));
			},
			None => {
				self.bounding_box.set(Some((pos, pos)));
			}
		}
		let out = self.voxels.insert(&pos, self.voxel_palette.get_palette_id(&voxel))?;
		self.voxel_palette.get_voxel(out).cloned()
	}

	pub fn remove_voxel(&mut self, pos: &I16Vec3) -> Option<Voxel> {
		self.bounding_box_dirty.set(true);
		self.voxel_palette.get_voxel(self.voxels.remove(pos)?).cloned()
	}

	pub fn get_voxel(&self, pos: &I16Vec3) -> Option<&Voxel> {
			self.voxel_palette.get_voxel(self.voxels.get(pos)?)
		}
	pub fn get_voxels(&self) -> &GridTree { &self.voxels }
	pub fn get_palette(&self) -> &VoxelPalette { &self.voxel_palette }

	pub fn get_bounding_box(&self) -> Option<(I16Vec3, I16Vec3)> {
		if self.bounding_box_dirty.get() {
			let _zone = span!("rebuild voxel bounding box");
			self.bounding_box_dirty.set(false);
			self.bounding_box.set(self.voxels.iter().fold(None, |bb, (p, size, _)| {
				match bb {
					Some((min, max)) => Some((min.min(p), max.max(p + size as i16 - 1))),
					None => Some((p, p + size as i16 - 1))
				}
			}));
		}
		self.bounding_box.get()
	}
}
