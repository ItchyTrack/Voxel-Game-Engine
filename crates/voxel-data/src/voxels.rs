use bevy::math::{IVec2, IVec3, UVec3, Vec3};
use serde::{Deserialize, Serialize};
use tracy_client::span;
use std::{io::{self, Read, Write}, sync::{Mutex, atomic::{AtomicBool, Ordering}}};

use super::{grid_tree::{self, AsGridData, GridReducer, NonZeroVoxelRegion, SourceOverlaps as GridSourceOverlaps}, sdf::Sdf, voxel_grid_tree::{VoxelGridTree, VoxelGridType}};

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

pub trait VoxelType: bytemuck::Pod + bytemuck::Zeroable + Sized + Send + Sync + 'static {
	const TYPE_ID: VoxelTypeId;
	const TYPE_INFO: VoxelTypeInfo = VoxelTypeInfo {
			id: Self::TYPE_ID,
			size_bytes: std::mem::size_of::<Self>() as u16,
	};

	fn get_ref(&self) -> VoxelRef<'_> {
		VoxelRef::new(Self::TYPE_ID, bytemuck::bytes_of(self))
	}

	fn into_voxel(self) -> Voxel {
		Voxel::new(Self::TYPE_ID, bytemuck::bytes_of(&self))
	}

	fn from_voxel_ref(voxel: &VoxelRef) -> Self {
		Self::TYPE_ID.assert_type(voxel.type_id());
		bytemuck::pod_read_unaligned(voxel.bytes())
	}

	fn from_voxel(voxel: &Voxel) -> Self {
		Self::TYPE_ID.assert_type(voxel.type_id());
		bytemuck::pod_read_unaligned(voxel.bytes())
	}
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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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
	pub fn bytes(&self) -> &'bytes [u8] { self.bytes }

	pub fn assert_type<T: VoxelType>(&self) {
		T::TYPE_INFO.id.assert_type(self.type_id());
	}

	pub fn get_voxel(&self) -> Voxel {
		Voxel::new(self.type_id, self.bytes())
	}
}

pub struct SourceTree<'a> {
	pub voxels: &'a Voxels,

	/// Source-local coordinates are divided by `1 << scale_down`.
	pub scale_down: u8,

	/// Added after scale-down to place source data in output space.
	pub output_offset: IVec3,
}

pub struct SourceOverlap<'a> {
	pub source_index: usize,

	/// Source-local region contributing to the current output region.
	pub source_region: NonZeroVoxelRegion,

	/// Projected/clipped region in output coordinates.
	pub output_region: NonZeroVoxelRegion,

	pub data: VoxelRef<'a>,
}

pub struct SourceOverlaps<'overlaps, 'a> {
	overlaps: GridSourceOverlaps<'overlaps, 'a, VoxelGridType>,
}

impl<'overlaps, 'a> Iterator for SourceOverlaps<'overlaps, 'a> {
	type Item = SourceOverlap<'a>;

	fn next(&mut self) -> Option<Self::Item> {
		self.overlaps.next().map(|overlap| SourceOverlap {
			source_index: overlap.source_index,
			source_region: overlap.source_region,
			output_region: overlap.output_region,
			data: overlap.data,
		})
	}
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Voxels {
	voxels: VoxelGridTree,
	#[serde(skip, default = "default_bounding_box")]
	bounding_box: Mutex<Option<(UVec3, UVec3)>>,
	#[serde(skip, default = "default_bounding_box_dirty")]
	bounding_box_dirty: AtomicBool,
}

fn default_bounding_box() -> Mutex<Option<(UVec3, UVec3)>> { Mutex::new(None) }
fn default_bounding_box_dirty() -> AtomicBool { AtomicBool::new(true) }

impl Clone for Voxels {
	fn clone(&self) -> Self {
		Self {
			voxels: self.voxels.clone(),
			bounding_box: Mutex::new(*self.bounding_box.lock().unwrap()),
			bounding_box_dirty: AtomicBool::new(self.bounding_box_dirty.load(Ordering::Relaxed)),
		}
	}
}

pub trait VoxelReducer {
	type Output: VoxelType;

	fn output_type_info(&self) -> VoxelTypeInfo {
		Self::Output::TYPE_INFO
	}

	fn reduce<'overlaps, 'a>(
		&mut self,
		region: NonZeroVoxelRegion,
		overlaps: GridSourceOverlaps<'overlaps, 'a, VoxelGridType>,
	) -> Option<Self::Output>;
}

impl<R> GridReducer<VoxelGridType> for R
where
	R: VoxelReducer,
	for<'d> &'d R::Output: AsGridData<'d, VoxelGridType>,
{
	type Output = R::Output;

	fn output_grid_type(&self) -> VoxelGridType {
		VoxelGridType::new(self.output_type_info())
	}

	fn reduce<'overlaps, 'a>(
		&mut self,
		region: NonZeroVoxelRegion,
		overlaps: GridSourceOverlaps<'overlaps, 'a, VoxelGridType>,
	) -> Option<Self::Output>
	{
		VoxelReducer::reduce(self, region, overlaps)
	}
}

impl Voxels {
	pub fn new<T: VoxelType>() -> Self {
		Self::new_with_type(T::TYPE_INFO)
	}

	pub fn new_with_type(voxel_type: VoxelTypeInfo) -> Self {
		Self { voxels: VoxelGridTree::new_with_type(VoxelGridType::new(voxel_type)), bounding_box: Mutex::new(None), bounding_box_dirty: AtomicBool::new(false) }
	}

	pub fn reduce_voxels<'a, R>(output_region: NonZeroVoxelRegion, sources: &[SourceTree<'a>], reducer: R) -> Option<Self>
	where
		R: VoxelReducer,
		for<'d> &'d R::Output: AsGridData<'d, VoxelGridType>,
	{
		let source_trees: Vec<_> = sources
			.iter()
			.map(|source| grid_tree::SourceTree {
				tree: source.voxels.grid_tree(),
				scale_down: source.scale_down,
				output_offset: source.output_offset,
			})
			.collect();
		let voxels = grid_tree::reduce_grid_trees(
			output_region,
			&source_trees,
			reducer,
		)?;
		Some(Self {
			voxels,
			bounding_box: Mutex::new(None),
			bounding_box_dirty: AtomicBool::new(true),
		})
	}

	pub fn voxel_type_info(&self) -> VoxelTypeInfo { self.voxels.grid_type().type_info() }
	pub fn voxel_type_id(&self) -> VoxelTypeId { self.voxel_type_info().id }
	pub fn voxel_type_size_bytes(&self) -> u16 { self.voxel_type_info().size_bytes }

	pub fn assert_type(&self, voxel_type: VoxelTypeId) {
		self.voxel_type_id().assert_type(voxel_type);
	}

	pub fn add_voxel(&mut self, pos: UVec3, voxel: VoxelRef) -> bool {
		self.assert_type(voxel.type_id());
		let bb = *self.bounding_box.lock().unwrap();
		*self.bounding_box.get_mut().unwrap() = Some(match bb {
			Some((min, max)) => (min.min(pos), max.max(pos)),
			None => (pos, pos),
		});
		self.voxels.insert(&pos, voxel)
	}

	pub fn add_voxe_get_replaced(&mut self, pos: UVec3, voxel: VoxelRef, out_voxel_bytes: &mut [u8]) -> bool {
		self.assert_type(voxel.type_id());
		let bb = *self.bounding_box.lock().unwrap();
		*self.bounding_box.get_mut().unwrap() = Some(match bb {
			Some((min, max)) => (min.min(pos), max.max(pos)),
			None => (pos, pos),
		});
		let old = self.voxels.get(&pos).map(|v| v.bytes().to_vec());
		let replaced = self.voxels.insert(&pos, voxel);
		if let Some(bytes) = old {
			out_voxel_bytes.copy_from_slice(&bytes);
			replaced
		} else {
			false
		}
	}

	pub fn ensure_area_covered(&mut self, pos: UVec3, size: UVec3) -> bool {
		self.voxels.ensure_area_covered(&pos, size)
	}

	pub fn add_area(&mut self, pos: UVec3, size: UVec3, voxel: VoxelRef) {
		if size == UVec3::ZERO { return; }
		self.assert_type(voxel.type_id());
		let max = pos + size - UVec3::ONE;
		self.add_tree_areas(&vec![(pos, size, voxel)], Some((pos, max)));
	}

	pub fn add_voxels<'a>(&mut self, voxels: &[(UVec3, VoxelRef<'a>)]) {
		if voxels.is_empty() { return; }
		let (min, max) = voxels.iter().fold((UVec3::splat(u32::MAX), UVec3::ZERO), |(mn, mx), (pos, _)| (mn.min(*pos), mx.max(*pos)));
		self.add_voxels_in_bounds(voxels, min, max);
	}

	pub fn add_voxels_in_bounds<'a>(&mut self, voxels: &[(UVec3, VoxelRef<'a>)], min: UVec3, max: UVec3) {
		if voxels.is_empty() { return; }
		for (_, voxel) in voxels {
			self.assert_type(voxel.type_id());
		}
		self.voxels.add_single_voxels_in_bounds(voxels, min, max);
		let bb = self.bounding_box.get_mut().unwrap();
		*bb = Some(match *bb { Some((mn, mx)) => (mn.min(min), mx.max(max)), None => (min, max) });
	}

	pub fn add_areas<'a>(&mut self, areas: &[(UVec3, UVec3, VoxelRef<'a>)]) {
		let mut bounds: Option<(UVec3, UVec3)> = None;
		for (pos, size, voxel) in areas {
			if *size == UVec3::ZERO { continue; }
			self.assert_type(voxel.type_id());
			let max = *pos + size - UVec3::ONE;
			bounds = Some(match bounds { Some((mn, mx)) => (mn.min(*pos), mx.max(max)), None => (*pos, max) });
		}
		self.add_tree_areas(areas, bounds);
	}

	fn add_tree_areas<'a>(&mut self, tree_areas: &[(UVec3, UVec3, VoxelRef<'a>)], bounds: Option<(UVec3, UVec3)>) {
		if tree_areas.is_empty() { return; }
		self.voxels.add_areas(&tree_areas);
		let Some((min, max)) = bounds else { return };
		let bb = self.bounding_box.get_mut().unwrap();
		*bb = Some(match *bb { Some((mn, mx)) => (mn.min(min), mx.max(max)), None => (min, max) });
	}

	pub fn remove_voxel(&mut self, pos: &UVec3) -> bool {
		let out = self.voxels.remove(pos);
		self.bounding_box_dirty.store(true, Ordering::Release);
		out
	}

	pub fn remove_voxel_get_removed(&mut self, pos: &UVec3, out_voxel_bytes: &mut [u8]) -> bool {
		let old = self.voxels.get(pos).map(|v| v.bytes().to_vec());
		if self.voxels.remove(pos) {
			self.bounding_box_dirty.store(true, Ordering::Release);
			if let Some(bytes) = old {
				out_voxel_bytes.copy_from_slice(&bytes);
				true
			} else { false }
		} else {
			false
		}
	}

	pub fn remove_area(&mut self, pos: UVec3, size: UVec3) {
		let Some(region) = NonZeroVoxelRegion::from_min_size(pos.as_ivec3(), size) else { return };
		self.voxels.clear_region(region);
		self.bounding_box_dirty.store(true, Ordering::Release);
	}

	pub fn apply_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize, voxel: VoxelRef) {
		self.assert_type(voxel.type_id());
		self.voxels.apply_sdf(initial_min, initial_max, sdf, face_resolution, iterations, voxel);
		self.bounding_box_dirty.store(true, Ordering::Release);
	}

	pub fn clear_sdf(&mut self, initial_min: Vec3, initial_max: Vec3, sdf: &(impl Sdf + ?Sized), face_resolution: IVec2, iterations: usize) {
		self.voxels.clear_sdf(initial_min, initial_max, sdf, face_resolution, iterations);
		self.bounding_box_dirty.store(true, Ordering::Release);
	}

	pub fn merge_from(&mut self, source: &Voxels, offset: IVec3) { self.merge_region_from(source, None, offset); }

	pub fn merge_region_from(&mut self, source: &Voxels, source_region: Option<NonZeroVoxelRegion>, offset: IVec3) {
		self.assert_type(source.voxel_type_id());
		let bounds = match source_region {
			Some(region) => source.voxels.occupied_bounds_in_region(region),
			None => source.voxels.occupied_bounds(),
		}.map(|region| ((region.min() + offset).as_uvec3(), (region.end() + offset - IVec3::ONE).as_uvec3()));
		match source_region {
			Some(region) => self.voxels.merge_region_from(&source.voxels, region, offset),
			None => self.voxels.merge_tree(&source.voxels, offset),
		}
		if let Some((min, max)) = bounds {
			let bb = self.bounding_box.get_mut().unwrap();
			*bb = Some(match *bb { Some((lo, hi)) => (lo.min(min), hi.max(max)), None => (min, max) });
		}
	}

	pub fn overwrite_region_from(&mut self, source: &Voxels, source_region: NonZeroVoxelRegion, offset: IVec3) {
		self.assert_type(source.voxel_type_id());
		self.voxels.overwrite_region_from(&source.voxels, source_region, offset);
		self.bounding_box_dirty.store(true, Ordering::Release);
	}

	pub fn voxel(&self, pos: &UVec3) -> Option<VoxelRef<'_>> { self.voxels.get(pos) }
	pub fn raw(&self, pos: &UVec3) -> Option<&[u8]> {
		let voxel = self.voxels.get(pos)?;
		Some(voxel.bytes())
	}
	pub fn grid_tree(&self) -> &VoxelGridTree { &self.voxels }
	pub fn is_empty(&self) -> bool { self.voxels.len() == 0 }

	pub fn bounding_box(&self) -> Option<(UVec3, UVec3)> {
		if self.bounding_box_dirty.load(Ordering::Acquire) {
			let _zone = span!("rebuild voxel bounding box");
			let bounds = self.voxels.occupied_bounds().map(|region| (region.min().as_uvec3(), (region.end() - IVec3::ONE).as_uvec3()));
			self.bounding_box_dirty.store(false, Ordering::Release);
			*(self.bounding_box.lock()).unwrap() = bounds;
		}
		*self.bounding_box.lock().unwrap()
	}

	pub fn write_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
		let type_info = self.voxel_type_info();
		writer.write_all(&type_info.id.0.to_le_bytes())?;
		writer.write_all(&type_info.size_bytes.to_le_bytes())?;
		self.voxels.write_to(writer)
	}

	pub fn read_from<R: Read>(reader: &mut R) -> io::Result<Self> {
		let mut type_id_buf = [0u8; 2];
		let mut type_size_buf = [0u8; 2];
		reader.read_exact(&mut type_id_buf)?;
		reader.read_exact(&mut type_size_buf)?;
		let type_info = VoxelTypeInfo { id: VoxelTypeId(u16::from_le_bytes(type_id_buf)), size_bytes: u16::from_le_bytes(type_size_buf) };
		let voxels = VoxelGridTree::read_from_with_type(VoxelGridType::new(type_info), reader)?;
		Ok(Self {
			voxels,
			bounding_box: Mutex::new(None),
			bounding_box_dirty: AtomicBool::new(true),
		})
	}
}
