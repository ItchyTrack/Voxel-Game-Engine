use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Res, ResMut};
use bevy::render::renderer::RenderDevice;

use bevy::prelude::Entity;
use crate::incoming_ray_directions::IncomingRayDirections;

use crate::gpu_bvh::GpuBvh;

#[derive(Resource, Clone, Default)]
pub struct DirectionFeedback(pub HashMap<Entity, IncomingRayDirections>);

#[derive(Resource, Default)]
pub struct LastGpuBvh(pub Mutex<Option<GpuBvh<Entity>>>);

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

const READBACK_TIMEOUT: Duration = Duration::from_millis(100);

/// Read back the previous frame's packed per-item local direction masks.
pub fn read_back_direction_masks(
	render_device: Res<RenderDevice>,
	last_gpu_bvh: ResMut<LastGpuBvh>,
	mut feedback: ResMut<DirectionFeedback>,
) {
	let Some(prev) = last_gpu_bvh.0.lock().ok().and_then(
		|mut slot| slot.take()
	) else { return };

	let (tx, rx) = std::sync::mpsc::channel();
	{
		let staging = prev.item_direction_mask_staging_buffer.slice(..);
		staging.map_async(wgpu::MapMode::Read, move |result| { let _ = tx.send(result); });
	}

	let polled = render_device.wgpu_device().poll(wgpu::PollType::Wait {
		submission_index: None,
		timeout: Some(READBACK_TIMEOUT),
	});

	if polled.is_err() || !matches!(rx.try_recv(), Ok(Ok(()))) {
		return;
	}

	{
		let staging = prev.item_direction_mask_staging_buffer.slice(..);
		let view = staging.get_mapped_range();
		let words: &[u32] = bytemuck::cast_slice(&view);
		let n = prev.item_count.min(prev.item_ids.len());

		feedback.0.clear();
		for (item_index, id) in prev.item_ids[..n].iter().enumerate() {
			let word = words[item_index / 4];
			let shift = (item_index % 4) * 8;
			feedback.0.insert(*id, IncomingRayDirections::from_bits_truncate((word >> shift) as u8));
		}
		drop(view);
		prev.item_direction_mask_staging_buffer.unmap();
	}
}
