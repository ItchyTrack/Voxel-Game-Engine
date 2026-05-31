use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Res, ResMut};
use bevy::render::renderer::RenderDevice;

use voxel_bvh::gpu_bvh::GpuBvh;
use voxel_data::subgrid::SubGridId;

#[derive(Resource, Clone, Default)]
pub struct HitCountFeedback(pub Arc<Mutex<HashMap<SubGridId, u32>>>);

#[derive(Resource, Default)]
pub struct LastGpuBvh(pub Mutex<Option<GpuBvh<SubGridId>>>);

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

/// Read back the previous frame's per-item hit-count buffer.
pub fn read_back_hit_counts(
	render_device: Res<RenderDevice>,
	last_gpu_bvh: ResMut<LastGpuBvh>,
	feedback: Res<HitCountFeedback>,
) {
	let Ok(mut slot) = last_gpu_bvh.0.lock() else { return };
	let Some(prev) = slot.take() else { return };

	let staging = prev.item_hit_count_staging_buffer.slice(..);
	staging.map_async(wgpu::MapMode::Read, |_| {});
	let _ = render_device.wgpu_device().poll(wgpu::PollType::Wait {
		submission_index: None,
		timeout: None,
	});

	let view = staging.get_mapped_range();
	let counts: &[u32] = bytemuck::cast_slice(&view);

	let Ok(mut feedback) = feedback.0.lock() else { return };
	feedback.clear();
	for (id, count) in prev.item_ids.iter().zip(&counts[..prev.item_count]) {
		feedback.insert(*id, *count);
	}
	drop(view);
	prev.item_hit_count_staging_buffer.unmap();
}
