use std::{collections::HashMap, sync::Arc};

use bevy::prelude::*;
use voxel_data::voxels::{VoxelRef, VoxelType, VoxelTypeId};

type VoxelColorReaderFn = Arc<dyn for<'bytes> Fn(&VoxelRef<'bytes>) -> Option<[u8; 4]> + Send + Sync>;
type VoxelColorBulkReaderFn = Arc<dyn for<'bytes> Fn(Vec<VoxelRef<'bytes>>) -> Vec<[u8; 4]> + Send + Sync>;

#[derive(Clone)]
struct VoxelColorReader {
	color: VoxelColorReaderFn,
	colors: VoxelColorBulkReaderFn,
}

#[derive(Resource, Default, Clone)]
pub struct VoxelColorReaders {
	readers: HashMap<VoxelTypeId, VoxelColorReader>,
}

impl VoxelColorReaders {
	pub fn register<T: VoxelColor>(&mut self) {
		self.readers.insert(T::TYPE_INFO.id, VoxelColorReader {
			color: Arc::new(|voxel| Some(T::from_voxel_ref(voxel).voxel_color())),
			colors: Arc::new(|voxels| T::voxel_color_bulk(voxels.iter().map(T::from_voxel_ref).collect())),
		});
	}

	pub fn color(&self, voxel: &VoxelRef<'_>) -> Option<[u8; 4]> {
		self.readers.get(&voxel.type_id()).and_then(|reader| (reader.color)(voxel))
	}

	pub fn colors(&self, type_id: VoxelTypeId, voxels: Vec<VoxelRef<'_>>) -> Option<Vec<[u8; 4]>> {
		Some((self.readers.get(&type_id)?.colors)(voxels))
	}
}

pub trait VoxelColor: VoxelType {
	fn voxel_color(&self) -> [u8; 4];
	fn voxel_color_bulk(voxels: Vec<Self>) -> Vec<[u8; 4]> {
		voxels.iter().map(Self::voxel_color).collect()
	}
}

pub trait VoxelGpuAppExt {
	fn register_voxel_color<T: VoxelColor>(&mut self) -> &mut Self;
}

impl VoxelGpuAppExt for App {
	fn register_voxel_color<T: VoxelColor>(&mut self) -> &mut Self {
		if !self.world().contains_resource::<VoxelColorReaders>() {
			self.init_resource::<VoxelColorReaders>();
		}
		self.world_mut().resource_mut::<VoxelColorReaders>().register::<T>();
		self
	}
}
