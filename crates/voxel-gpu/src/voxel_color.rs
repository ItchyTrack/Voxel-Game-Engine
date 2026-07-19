use std::{collections::HashMap, sync::Arc};

use bevy::prelude::*;
use voxel_data::voxels::{Voxel, VoxelType, VoxelTypeId};

type VoxelColorReader = Arc<dyn Fn(&Voxel) -> Option<[u8; 4]> + Send + Sync>;

#[derive(Resource, Default, Clone)]
pub struct VoxelColorReaders {
	readers: HashMap<VoxelTypeId, VoxelColorReader>,
}

impl VoxelColorReaders {
	pub fn register<T: VoxelType>(&mut self, color: impl Fn(&T) -> [u8; 4] + Send + Sync + 'static) {
		self.readers.insert(T::TYPE_INFO.id, Arc::new(move |voxel| Some(color(&T::from_voxel(voxel)))));
	}

	pub fn color(&self, voxel: &Voxel) -> Option<[u8; 4]> {
		self.readers.get(&voxel.type_id()).and_then(|reader| reader(voxel))
	}
}

pub trait VoxelGpuAppExt {
	fn register_voxel_color<T: VoxelType>(&mut self, color: impl Fn(&T) -> [u8; 4] + Send + Sync + 'static) -> &mut Self;
}

impl VoxelGpuAppExt for App {
	fn register_voxel_color<T: VoxelType>(&mut self, color: impl Fn(&T) -> [u8; 4] + Send + Sync + 'static) -> &mut Self {
		if !self.world().contains_resource::<VoxelColorReaders>() {
			self.init_resource::<VoxelColorReaders>();
		}
		self.world_mut().resource_mut::<VoxelColorReaders>().register::<T>(color);
		self
	}
}
