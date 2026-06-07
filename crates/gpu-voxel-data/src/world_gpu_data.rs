use bevy::ecs::{resource::Resource, world::FromWorld};
use bevy::render::renderer::{RenderDevice, RenderQueue};

use crate::packed_dynamic_buffer::PackedDynamicBuffer;

#[derive(Resource, Debug)]
pub struct WorldGpuData {
	pub packed_64_tree_dynamic_buffer: PackedDynamicBuffer,
	pub packed_voxel_data_dynamic_buffer: PackedDynamicBuffer,
}

impl FromWorld for WorldGpuData {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let render_queue = world.resource::<RenderQueue>();

		let packed_64_tree_dynamic_buffer = PackedDynamicBuffer::new(
			render_device,
			render_queue,
			12,
			wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
		)
		.expect("Failed to create packed_64_tree_dynamic_buffer");

		let packed_voxel_data_dynamic_buffer = PackedDynamicBuffer::new(
			render_device,
			render_queue,
			4,
			wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
		)
		.expect("Failed to create packed_voxel_data_dynamic_buffer");

		Self {
			packed_64_tree_dynamic_buffer: packed_64_tree_dynamic_buffer,
			packed_voxel_data_dynamic_buffer: packed_voxel_data_dynamic_buffer,
		}
	}
}
