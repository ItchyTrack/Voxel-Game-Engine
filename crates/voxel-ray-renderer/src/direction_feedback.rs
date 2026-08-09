use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use bevy::asset::{AssetEvent, Assets, RenderAssetUsages};
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Commands, Query, ResMut};
use bevy::prelude::{Component, Entity, On};
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::storage::ShaderBuffer;
use bevy::render::MainWorld;

use crate::extract::ExtractedVoxelScene;
use crate::incoming_ray_directions::IncomingRayDirections;

#[derive(Resource, Clone, Default)]
pub struct DirectionFeedback(pub HashMap<Entity, IncomingRayDirections>);

#[derive(Component)]
struct DirectionMaskReadback {
	item_ids: Vec<Entity>,
	completed: bool,
}

/// GPU-side render stats published by the render world for the main world to read.
#[derive(Resource, Clone, Default)]
pub struct RenderStats {
	pub inner: Arc<Mutex<RenderStatsData>>,
}

#[derive(Default, Clone, Copy, Debug)]
pub struct RenderStatsData {
	pub bvh_bytes: u64,
	pub bvh_leaf_bytes: u64,
}

/// Create a GPU buffer and an entity that will asynchronously read its direction masks back.
pub fn prepare_direction_mask_readback(
	mut main_world: ResMut<MainWorld>,
	mut extracted_scenes: Query<&mut ExtractedVoxelScene>,
) {
	for mut extracted in &mut extracted_scenes {
		let Some(item_ids) = extracted.bvh.as_ref().map(|bvh| {
			bvh.internals().1.iter().map(|item| item.0).collect::<Vec<_>>()
		}) else { continue };

		let direction_mask_size = item_ids.len().div_ceil(4) * size_of::<u32>();
		let mut buffer = ShaderBuffer::with_size(direction_mask_size, RenderAssetUsages::RENDER_WORLD);
		buffer.buffer_description.label = Some("bvh_item_direction_mask_buffer");
		buffer.buffer_description.usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC;

		let handle = main_world.resource_mut::<Assets<ShaderBuffer>>().add(buffer);
		main_world.write_message(AssetEvent::Added { id: handle.id() });
		extracted.direction_mask_buffer = Some(handle.clone());
		main_world
			.spawn((Readback::buffer(handle), DirectionMaskReadback { item_ids, completed: false }))
			.observe(read_back_direction_masks);
	}
}

fn read_back_direction_masks(
	event: On<ReadbackComplete>,
	mut readbacks: Query<&mut DirectionMaskReadback>,
	mut feedback: ResMut<DirectionFeedback>,
	mut commands: Commands,
) {
	let Ok(mut readback) = readbacks.get_mut(event.entity) else { return };
	if readback.completed { return; }
	readback.completed = true;

	feedback.0.clear();
	for (item_index, id) in readback.item_ids.iter().enumerate() {
		let byte_index = (item_index / 4) * size_of::<u32>();
		let Some(bytes) = event.data.get(byte_index..byte_index + size_of::<u32>()) else { break };
		let word = u32::from_ne_bytes(bytes.try_into().expect("direction mask word has four bytes"));
		let shift = (item_index % 4) * 8;
		feedback.0.insert(*id, IncomingRayDirections::from_bits_truncate((word >> shift) as u8));
	}

	commands.entity(event.entity).despawn();
}
