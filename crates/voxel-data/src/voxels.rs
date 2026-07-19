use bevy::math::{IVec2, IVec3, U16Vec3, Vec3};
use serde::{Deserialize, Serialize};
use tracy_client::span;
use std::{io::{self, Read, Write}, sync::{Mutex, atomic::{AtomicBool, Ordering}}};
use bimap::BiHashMap;

use super::{grid_tree::{GridRegion, size as grid_tree_size}, sdf::Sdf, voxel_grid_tree::VoxelGridTree};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct VoxelTypeId(pub u16);

impl VoxelTypeId {
	pub fn assert_type(self, other: VoxelTypeId) {
		assert_eq!(self, other);
	}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct VoxelTypeInfo {
	pub id: VoxelTypeId,
	pub size_bytes: u16,
}

pub trait VoxelType: Sized + 'static {
	const TYPE_INFO: VoxelTypeInfo;

	fn into_voxel(self) -> Voxel;
	fn from_voxel(voxel: &Voxel) -> Self;
	fn from_voxel_ref(voxel: &VoxelRef) -> Self;
	fn into_bytes(self, bytes: &mut [u8]);
	fn from_bytes(bytes: &[u8]) -> Self;
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Voxel {
	type_id: VoxelTypeId,
	bytes: Box<[u8]>,
}

impl Voxel {
	pub fn new(type_id: VoxelTypeId, bytes: impl Into<Box<[u8]>>) -> Self {
		Self { type_id, bytes: bytes.into() }
	}

	pub fn type_id(&self) -> VoxelTypeId { self.type_id }
	pub fn size(&self) -> u16 { self.bytes.len() as u16 }
	pub fn type_info(&self) -> VoxelTypeInfo { VoxelTypeInfo { id: self.type_id, size_bytes: self.size() as u16 } }
	pub fn bytes(&self) -> &[u8] { &self.bytes }

	pub fn assert_type<T: VoxelType>(&self) {
		T::TYPE_INFO.id.assert_type(self.type_id());
	}

	pub fn get_ref(&self) -> VoxelRef<'_> {
		VoxelRef::new(self.type_id, self.bytes())
	}
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct VoxelRef<'bytes> {
	type_id: VoxelTypeId,
	bytes: &'bytes [u8],
}

impl<'bytes> VoxelRef<'bytes> {
	pub fn new(type_id: VoxelTypeId, bytes: &'bytes [u8]) -> Self {
		Self { type_id, bytes }
	}

	pub fn type_id(&self) -> VoxelTypeId { self.type_id }
	pub fn size(&self) -> u16 { self.bytes.len() as u16 }
	pub fn type_info(&self) -> VoxelTypeInfo { VoxelTypeInfo { id: self.type_id, size_bytes: self.size() as u16 } }
	pub fn bytes(&self) -> &[u8] { &self.bytes }

	pub fn assert_type<T: VoxelType>(&self) {
		T::TYPE_INFO.id.assert_type(self.type_id());
	}

	pub fn get_voxel(&self) -> Voxel {
		Voxel::new(self.type_id, self.bytes())
	}
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VoxelPalette {
	type_info: VoxelTypeInfo,
	bytes: Vec<u8>,
	entries: BiHashMap<u16, u8>,
	next_id: u16,
}

impl VoxelPalette {
	pub fn new<T: VoxelType>() -> Self {
		Self::new_with_type(T::TYPE_INFO)
	}
	
	pub fn new_with_type(type_info: VoxelTypeInfo) -> Self {
		Self { type_info, bytes: Vec::new(), entries: BiHashMap::new(), next_id: 0 }
	}

	pub fn type_info(&self) -> VoxelTypeInfo { self.type_info }

	pub fn assert_type(&self, voxel_type: VoxelTypeId) {
		self.type_info.id.assert_type(voxel_type);
	}

	pub fn entries(&self) -> impl Iterator<Item = (u16, &[u8])> + '_ {
		self.entries.iter().filter_map(|(id, index)| self.raw_at_index(*index).map(|raw| (*id, raw)))
	}

	pub fn palette_id(&mut self, voxel: VoxelRef) -> u16 {
		self.assert_type(voxel.type_info().id);
		self.palette_id_from_bytes(voxel.bytes())
	}

	fn palette_id_from_bytes(&mut self, bytes: &[u8]) -> u16 {
		for (id, index) in self.entries.iter() {
			if self.raw_at_index(*index) == Some(bytes) {
				return *id;
			}
		}
		let index = (self.bytes.len() as u32 / self.type_info.size_bytes as u32) as u8;
		let id = self.next_id;
		self.next_id += 1;
		self.bytes.extend_from_slice(bytes);
		self.entries.insert_no_overwrite(id, index).expect("duplicate voxel palette id or index");
		id
	}

	pub fn voxel(&self, id: u16) -> Option<VoxelRef<'_>> {
		Some(VoxelRef::new(self.type_info().id, self.raw(id)?))
	}

	pub fn raw(&self, id: u16) -> Option<&[u8]> {
		self.raw_at_index(*self.entries.get_by_left(&id)?)
	}

	fn raw_at_index(&self, index: u8) -> Option<&[u8]> {
		let start = index as u32 * self.type_info.size_bytes as u32;
		let end = start + self.type_info.size_bytes as u32;
		self.bytes.get(start as usize..end as usize)
	}

	fn write_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
		writer.write_all(&self.type_info.id.0.to_le_bytes())?;
		writer.write_all(&self.type_info.size_bytes.to_le_bytes())?;
		writer.write_all(&((self.bytes.len() as u32 / self.type_info.size_bytes as u32) as u8).to_le_bytes())?;
		writer.write_all(&self.bytes)?;
		writer.write_all(&(self.entries.len() as u32).to_le_bytes())?;
		writer.write_all(&self.next_id.to_le_bytes())?;
		for (id, index) in self.entries.iter() {
			writer.write_all(&id.to_le_bytes())?;
			writer.write_all(&[*index])?;
		}
		Ok(())
	}

	fn read_from<R: Read>(reader: &mut R) -> io::Result<Self> {
		let mut type_id_buf = [0u8; 2];
		let mut type_size_buf = [0u8; 2];
		reader.read_exact(&mut type_id_buf)?;
		reader.read_exact(&mut type_size_buf)?;
		let type_info = VoxelTypeInfo { id: VoxelTypeId(u16::from_le_bytes(type_id_buf)), size_bytes: u16::from_le_bytes(type_size_buf) };
		assert_ne!(type_info.size_bytes, 0);
		let mut bytes_len_buf = [0u8];
		reader.read_exact(&mut bytes_len_buf)?;
		let bytes_len = u8::from_le_bytes(bytes_len_buf) as usize;
		let mut bytes = vec![0u8; (bytes_len as u32 * type_info.size_bytes as u32) as usize];
		reader.read_exact(&mut bytes)?;
		let mut len_buf = [0u8; 4];
		reader.read_exact(&mut len_buf)?;
		let len = u32::from_le_bytes(len_buf) as usize;
		let mut next_id_buf = [0u8; 2];
		reader.read_exact(&mut next_id_buf)?;
		let next_id = u16::from_le_bytes(next_id_buf);
		let mut entries = BiHashMap::new();
		for _ in 0..len {
			let mut id_buf = [0u8; 2];
			let mut index_buf = [0u8; 1];
			reader.read_exact(&mut id_buf)?;
			reader.read_exact(&mut index_buf)?;
			let id = u16::from_le_bytes(id_buf);
			if entries.insert_no_overwrite(id, index_buf[0]).is_err() {
				return Err(io::Error::new(io::ErrorKind::InvalidData, "duplicate voxel palette id or index during decode"));
			}
		}
		Ok(Self { type_info, bytes, entries, next_id })
	}
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Voxels {
	voxels: VoxelGridTree,
	voxel_palette: VoxelPalette,
	#[serde(skip, default = "default_bounding_box")]
	bounding_box: Mutex<Option<(U16Vec3, U16Vec3)>>,
	#[serde(skip, default = "default_bounding_box_dirty")]
	bounding_box_dirty: AtomicBool,
}

fn default_bounding_box() -> Mutex<Option<(U16Vec3, U16Vec3)>> { Mutex::new(None) }
fn default_bounding_box_dirty() -> AtomicBool { AtomicBool::new(true) }

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
	pub fn new<T: VoxelType>() -> Self {
		Self::new_with_type(T::TYPE_INFO)
	}

	pub fn new_with_type(voxel_type: VoxelTypeInfo) -> Self {
		Self { voxels: VoxelGridTree::new(), voxel_palette: VoxelPalette::new_with_type(voxel_type), bounding_box: Mutex::new(None), bounding_box_dirty: AtomicBool::new(false) }
	}

	pub fn voxel_type_info(&self) -> VoxelTypeInfo { self.voxel_palette.type_info() }
	pub fn voxel_type_id(&self) -> VoxelTypeId { self.voxel_type_info().id }
	pub fn voxel_type_size_bytes(&self) -> u16 { self.voxel_type_info().size_bytes }

	pub fn assert_type(&self, voxel_type: VoxelTypeId) {
		self.voxel_type_id().assert_type(voxel_type);
	}

	pub fn add_voxel(&mut self, pos: U16Vec3, voxel: VoxelRef) -> bool {
		self.assert_type(voxel.type_id());
		let bb = *self.bounding_box.lock().unwrap();
		*self.bounding_box.get_mut().unwrap() = Some(match bb {
			Some((min, max)) => (min.min(pos), max.max(pos)),
			None => (pos, pos),
		});
		self.voxels.insert(&pos, self.voxel_palette.palette_id(voxel)).is_some()
	}
	
	pub fn add_voxe_get_replaced(&mut self, pos: U16Vec3, voxel: VoxelRef, out_voxel_bytes: &mut [u8]) -> bool {
		self.assert_type(voxel.type_id());
		let bb = *self.bounding_box.lock().unwrap();
		*self.bounding_box.get_mut().unwrap() = Some(match bb {
			Some((min, max)) => (min.min(pos), max.max(pos)),
			None => (pos, pos),
		});
		if let Some(out) = self.voxels.insert(&pos, self.voxel_palette.palette_id(voxel)) {
			if let Some(bytes) = self.raw_for_palette_id(out) {
				out_voxel_bytes.copy_from_slice(bytes);
				true
			} else {
				false
			}
		} else {
			false
		}
	}

	pub fn ensure_area_covered(&mut self, pos: U16Vec3, size: U16Vec3) -> bool {
		self.voxels.ensure_area_covered(&pos, size.as_ivec3())
	}

	pub fn add_area(&mut self, pos: U16Vec3, size: U16Vec3, voxel: VoxelRef) {
		if size == U16Vec3::ZERO { return; }
		self.assert_type(voxel.type_id());
		let id = self.voxel_palette.palette_id(voxel);
		let max = pos + size - U16Vec3::ONE;
		self.add_tree_areas(vec![(pos, size.as_ivec3(), id)], Some((pos, max)));
	}

	pub fn add_voxels(&mut self, voxels: &[(U16Vec3, u16)], source_palette: &VoxelPalette) {
		if voxels.is_empty() { return; }
		let (min, max) = voxels.iter().fold((U16Vec3::splat(u16::MAX), U16Vec3::ZERO), |(mn, mx), (pos, _)| (mn.min(*pos), mx.max(*pos)));
		self.add_voxels_in_bounds(voxels, source_palette, min, max);
	}

	pub fn add_voxels_in_bounds(&mut self, voxels: &[(U16Vec3, u16)], source_palette: &VoxelPalette, min: U16Vec3, max: U16Vec3) {
		if voxels.is_empty() { return; }
		let source_type = source_palette.type_info();
		self.voxel_palette.assert_type(source_type.id);
		if self.is_empty() {
			self.voxel_palette = source_palette.clone();
			self.voxels.add_single_voxels_in_bounds(voxels, min.as_ivec3(), max.as_ivec3());
		} else {
			let mut palette_cache: Vec<(u16, u16)> = Vec::new();
			let mapped: Vec<_> = voxels.iter().map(|(pos, source_id)| {
				let id = if let Some((_, id)) = palette_cache.iter().find(|(cached, _)| cached == source_id) { *id } else {
					let raw = source_palette.raw(*source_id).expect("source palette id missing");
					let id = self.voxel_palette.palette_id_from_bytes(raw);
					palette_cache.push((*source_id, id));
					id
				};
				(*pos, id)
			}).collect();
			self.voxels.add_single_voxels_in_bounds(&mapped, min.as_ivec3(), max.as_ivec3());
		}
		let bb = self.bounding_box.get_mut().unwrap();
		*bb = Some(match *bb { Some((mn, mx)) => (mn.min(min), mx.max(max)), None => (min, max) });
	}

	pub fn add_areas(&mut self, areas: &[(U16Vec3, U16Vec3, u16)], source_palette: &VoxelPalette) {
		let mut tree_areas = Vec::with_capacity(areas.len());
		let mut bounds: Option<(U16Vec3, U16Vec3)> = None;
		let source_type = source_palette.type_info();
		self.voxel_palette.assert_type(source_type.id);
		if self.is_empty() {
			self.voxel_palette = source_palette.clone();
			for (pos, size, id) in areas {
				if *size == U16Vec3::ZERO { continue; }
				tree_areas.push((*pos, size.as_ivec3(), *id));
				let max = *pos + *size - U16Vec3::ONE;
				bounds = Some(match bounds { Some((mn, mx)) => (mn.min(*pos), mx.max(max)), None => (*pos, max) });
			}
		} else {
			let mut palette_cache: Vec<(u16, u16)> = Vec::new();
			for (pos, size, source_id) in areas {
				if *size == U16Vec3::ZERO { continue; }
				let id = if let Some((_, id)) = palette_cache.iter().find(|(cached, _)| cached == source_id) { *id } else {
					let raw = source_palette.raw(*source_id).expect("source palette id missing");
					let id = self.voxel_palette.palette_id_from_bytes(raw);
					palette_cache.push((*source_id, id));
					id
				};
				tree_areas.push((*pos, size.as_ivec3(), id));
				let max = *pos + *size - U16Vec3::ONE;
				bounds = Some(match bounds { Some((mn, mx)) => (mn.min(*pos), mx.max(max)), None => (*pos, max) });
			}
		}
		self.add_tree_areas(tree_areas, bounds);
	}

	fn add_tree_areas(&mut self, tree_areas: Vec<(U16Vec3, IVec3, u16)>, bounds: Option<(U16Vec3, U16Vec3)>) {
		if tree_areas.is_empty() { return; }
		self.voxels.add_areas(&tree_areas);
		let Some((min, max)) = bounds else { return };
		let bb = self.bounding_box.get_mut().unwrap();
		*bb = Some(match *bb { Some((mn, mx)) => (mn.min(min), mx.max(max)), None => (min, max) });
	}

	pub fn remove_voxel(&mut self, pos: &U16Vec3) -> bool {
		let out = self.voxels.remove(pos).is_some();
		self.bounding_box_dirty.store(true, Ordering::Release);
		out
	}

	pub fn remove_voxel_get_removed(&mut self, pos: &U16Vec3, out_voxel_bytes: &mut [u8]) -> bool {
		if let Some(id) = self.voxels.remove(pos) {
			self.bounding_box_dirty.store(true, Ordering::Release);
			if let Some(bytes) = self.raw_for_palette_id(id) {
				out_voxel_bytes.copy_from_slice(bytes);
				true
			} else {
				false
			}
		} else {
			false
		}
	}

	pub fn remove_area(&mut self, pos: U16Vec3, size: U16Vec3) {
		let Some(region) = GridRegion::from_min_size(pos.as_ivec3(), size.as_ivec3()) else { return };
		self.voxels.clear_region(region);
		self.bounding_box_dirty.store(true, Ordering::Release);
	}

	pub fn apply_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize, voxel: VoxelRef) {
		self.assert_type(voxel.type_id());
		let id = self.voxel_palette.palette_id_from_bytes(voxel.bytes());
		self.voxels.apply_sdf(initial_min, initial_max, sdf, face_resolution, iterations, id);
		self.bounding_box_dirty.store(true, Ordering::Release);
	}

	pub fn clear_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize) {
		self.voxels.clear_sdf(initial_min, initial_max, sdf, face_resolution, iterations);
		self.bounding_box_dirty.store(true, Ordering::Release);
	}

	pub fn merge_from(&mut self, source: &Voxels, offset: IVec3) { self.merge_region_from(source, None, offset); }

	pub fn merge_region_from(&mut self, source: &Voxels, source_region: Option<GridRegion>, offset: IVec3) {
		self.assert_type(source.voxel_type_id());
		let bounds = match source_region {
			Some(region) => source.voxels.occupied_bounds_in_region(region),
			None => source.voxels.occupied_bounds(),
		}.map(|region| ((region.min + offset).as_u16vec3(), (region.end + offset - IVec3::ONE).as_u16vec3()));
		if self.voxels.is_empty() {
			self.voxel_palette = source.voxel_palette.clone();
			match source_region {
				Some(region) => self.voxels.merge_region_from(&source.voxels, region, offset),
				None => self.voxels.merge_tree(&source.voxels, offset),
			}
		} else {
			let mut palette_cache: Vec<(u16, u16)> = Vec::new();
			let palette = &mut self.voxel_palette;
			let mut map_id = |source_id| {
				if let Some((_, id)) = palette_cache.iter().find(|(cached, _)| *cached == source_id) { *id } else {
					let raw = source.voxel_palette.raw(source_id).expect("source palette id missing");
					let id = palette.palette_id_from_bytes(raw);
					palette_cache.push((source_id, id));
					id
				}
			};
			match source_region {
				Some(region) => self.voxels.merge_region_from_mapped(&source.voxels, region, offset, &mut map_id),
				None => {
					let (_, root_pos, root_depth) = source.voxels.internals();
					let root = GridRegion { min: root_pos.as_ivec3(), end: root_pos.as_ivec3() + IVec3::splat(grid_tree_size(root_depth) as i32) };
					self.voxels.merge_region_from_mapped(&source.voxels, root, offset, &mut map_id);
				}
			}
		}
		if let Some((min, max)) = bounds {
			let bb = self.bounding_box.get_mut().unwrap();
			*bb = Some(match *bb { Some((lo, hi)) => (lo.min(min), hi.max(max)), None => (min, max) });
		}
	}

	pub fn voxel(&self, pos: &U16Vec3) -> Option<VoxelRef<'_>> { self.voxel_for_palette_id(self.voxels.get(pos)?) }
	pub fn voxel_for_palette_id(&self, id: u16) -> Option<VoxelRef<'_>> { self.voxel_palette.voxel(id) }
	pub fn raw(&self, pos: &U16Vec3) -> Option<&[u8]> { self.raw_for_palette_id(self.voxels.get(pos)?) }
	pub fn raw_for_palette_id(&self, id: u16) -> Option<&[u8]> { self.voxel_palette.raw(id) }
	pub fn grid_tree(&self) -> &VoxelGridTree { &self.voxels }
	pub fn palette(&self) -> &VoxelPalette { &self.voxel_palette }
	pub fn is_empty(&self) -> bool { self.voxels.len() == 0 }

	pub fn bounding_box(&self) -> Option<(U16Vec3, U16Vec3)> {
		if self.bounding_box_dirty.load(Ordering::Acquire) {
			let _zone = span!("rebuild voxel bounding box");
			let bounds = self.voxels.occupied_bounds().map(|region| (region.min.as_u16vec3(), (region.end - IVec3::ONE).as_u16vec3()));
			self.bounding_box_dirty.store(false, Ordering::Release);
			*(self.bounding_box.lock()).unwrap() = bounds;
		}
		*self.bounding_box.lock().unwrap()
	}

	pub fn write_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
		self.voxels.write_to(writer)?;
		self.voxel_palette.write_to(writer)
	}

	pub fn read_from<R: Read>(reader: &mut R) -> io::Result<Self> {
		let voxels = VoxelGridTree::read_from(reader)?;
		let voxel_palette = VoxelPalette::read_from(reader)?;
		Ok(Self {
			voxels,
			voxel_palette,
			bounding_box: Mutex::new(None),
			bounding_box_dirty: AtomicBool::new(true),
		})
	}
}
