use std::{collections::HashMap, sync::Arc};

use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use voxel_data::voxels::{VoxelRef, VoxelType, VoxelTypeId};

type VoxelGpuBytesWriterFn = Arc<dyn for<'bytes> Fn(&[VoxelRef<'bytes>], &mut [u8]) + Send + Sync>;

#[derive(Clone)]
struct VoxelGpuDataReader {
	gpu_size_bytes: usize,
	write_bytes: VoxelGpuBytesWriterFn,
	shader_source: &'static str,
	shader_sampler: &'static str,
}

pub type VoxelShaderRegistration = (VoxelTypeId, &'static str, &'static str);

#[derive(Resource, ExtractResource, Default, Clone)]
pub struct VoxelGpuDataReaders {
	readers: HashMap<VoxelTypeId, VoxelGpuDataReader>,
}

impl VoxelGpuDataReaders {
	pub fn register<T: VoxelGpuData>(&mut self) {
		let gpu_size_bytes = T::voxel_gpu_size_bytes();
		self.readers.insert(T::TYPE_INFO.id, VoxelGpuDataReader {
			gpu_size_bytes,
			write_bytes: Arc::new(move |voxels, bytes| {
				assert_eq!(bytes.len(), voxels.len() * gpu_size_bytes, "GPU voxel byte output size must match voxel count and stride");
				for (voxel, out) in voxels.iter().zip(bytes.chunks_exact_mut(gpu_size_bytes)) {
					T::from_voxel_ref(voxel).write_voxel_gpu_raw(out);
				}
			}),
			shader_source: T::shader_source(),
			shader_sampler: T::shader_sampler(),
		});
	}

	pub fn gpu_size_bytes(&self, type_id: VoxelTypeId) -> Option<usize> {
		Some(self.readers.get(&type_id)?.gpu_size_bytes)
	}

	pub fn write_bytes(&self, type_id: VoxelTypeId, voxels: &[VoxelRef<'_>], bytes: &mut [u8]) -> Option<()> {
		let reader = self.readers.get(&type_id)?;
		assert_eq!(bytes.len(), voxels.len() * reader.gpu_size_bytes, "GPU voxel byte output size must match voxel count and stride");
		(reader.write_bytes)(voxels, bytes);
		Some(())
	}

	pub fn shader_sources(&self) -> Vec<VoxelShaderRegistration> {
		let mut sources = self.readers.iter()
			.map(|(type_id, reader)| (*type_id, reader.shader_source, reader.shader_sampler))
			.collect::<Vec<_>>();
		sources.sort_by_key(|source| source.0.0);
		sources
	}
}

pub trait VoxelGpuData: VoxelType {
	fn shader_source() -> &'static str;

	fn shader_sampler() -> &'static str;

	fn voxel_gpu_size_bytes() -> usize {
		std::mem::size_of::<Self>()
	}

	fn write_voxel_gpu_raw(&self, bytes: &mut [u8]) {
		let raw = bytemuck::bytes_of(self);
		assert_eq!(bytes.len(), raw.len(), "GPU voxel byte output size must match default raw size");
		bytes.copy_from_slice(raw);
	}
}

pub trait VoxelGpuAppExt {
	fn register_voxel_gpu_data<T: VoxelGpuData>(&mut self) -> &mut Self;
}

impl VoxelGpuAppExt for App {
	fn register_voxel_gpu_data<T: VoxelGpuData>(&mut self) -> &mut Self {
		if !self.world().contains_resource::<VoxelGpuDataReaders>() {
			self.init_resource::<VoxelGpuDataReaders>();
		}
		self.world_mut().resource_mut::<VoxelGpuDataReaders>().register::<T>();
		self
	}
}
