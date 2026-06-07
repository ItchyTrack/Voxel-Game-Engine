use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

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

const READBACK_TIMEOUT: Duration = Duration::from_millis(100);

/// Read back the previous frame's per-item hit-count buffer.
pub fn read_back_hit_counts(
	render_device: Res<RenderDevice>,
	last_gpu_bvh: ResMut<LastGpuBvh>,
	feedback: Res<HitCountFeedback>,
) {
	let Some(prev) = last_gpu_bvh.0.lock().ok().and_then(
		|mut slot| slot.take()
	) else { return };

	let staging = prev.item_hit_count_staging_buffer.slice(..);
	let (tx, rx) = std::sync::mpsc::channel();
	staging.map_async(wgpu::MapMode::Read, move |result| { let _ = tx.send(result); });

	let polled = render_device.wgpu_device().poll(wgpu::PollType::Wait {
		submission_index: None,
		timeout: Some(READBACK_TIMEOUT),
	});

	if polled.is_err() || !matches!(rx.try_recv(), Ok(Ok(()))) {
		return;
	}

	let view = staging.get_mapped_range();
	let counts: &[u32] = bytemuck::cast_slice(&view);
	let n = counts.len().min(prev.item_count).min(prev.item_ids.len());

	if let Ok(mut feedback) = feedback.0.lock() {
		feedback.clear();
		for (id, count) in prev.item_ids.iter().zip(&counts[..n]) {
			feedback.insert(*id, *count);
		}
	}
	drop(view);
	prev.item_hit_count_staging_buffer.unmap();
}
