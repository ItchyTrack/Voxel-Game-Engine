use std::{collections::HashMap, sync::Arc};

use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use voxel_data::voxels::{VoxelRef, VoxelType, VoxelTypeId};

type VoxelGpuEncoderFactoryFn = Arc<dyn for<'bytes> Fn(&[VoxelRef<'bytes>], &mut Vec<u8>) -> Box<dyn VoxelGpuBlockEncoder> + Send + Sync>;

#[derive(Clone)]
struct VoxelGpuDataReader {
	create_encoder: VoxelGpuEncoderFactoryFn,
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
		self.readers.insert(T::TYPE_INFO.id, VoxelGpuDataReader {
			create_encoder: Arc::new(move |voxels, header| Box::new(T::create_voxel_gpu_encoder(voxels, header))),
			shader_source: T::shader_source(),
			shader_sampler: T::shader_sampler(),
		});
	}

	pub fn create_encoder(&self, type_id: VoxelTypeId, voxels: &[VoxelRef<'_>], header: &mut Vec<u8>) -> Option<Box<dyn VoxelGpuBlockEncoder>> {
		let reader = self.readers.get(&type_id)?;
		Some((reader.create_encoder)(voxels, header))
	}

	pub fn shader_sources(&self) -> Vec<VoxelShaderRegistration> {
		let mut sources = self.readers.iter()
			.map(|(type_id, reader)| (*type_id, reader.shader_source, reader.shader_sampler))
			.collect::<Vec<_>>();
		sources.sort_by_key(|source| source.0.0);
		sources
	}
}

#[derive(Clone, Copy, Debug)]
pub enum VoxelGpuNodeEntry<'bytes> {
	/// A non-empty tree entry that points at a child node. It is included because
	/// shader voxel indices are ranks among all non-empty entries in the node, not
	/// only among data entries.
	ChildNode,
	/// A tree entry containing voxel data. The entry's slice has the registered
	/// voxel type.
	Data(VoxelRef<'bytes>),
}

pub trait VoxelGpuBlockEncoder: Send + Sync + 'static {
	/// Appends the encoded payload for one tree node. `entries` are ordered by the
	/// node bitmap. Encoders must make every [`VoxelGpuNodeEntry::Data`] readable
	/// by the same entry index the shader receives as `voxel_index`.
	fn write_node(&self, entries: &[VoxelGpuNodeEntry<'_>], out: &mut Vec<u8>);
}

pub trait VoxelGpuData: VoxelType {
	type Encoder: VoxelGpuBlockEncoder;

	fn shader_source() -> &'static str;

	fn shader_sampler() -> &'static str;

	/// Creates a block encoder for one uploaded voxel tree and writes optional
	/// block-local metadata to `header`. `voxel-gpu` owns all padding/alignment
	/// around this header before any per-node payload starts.
	fn create_voxel_gpu_encoder(voxels: &[VoxelRef<'_>], header: &mut Vec<u8>) -> Self::Encoder;
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
