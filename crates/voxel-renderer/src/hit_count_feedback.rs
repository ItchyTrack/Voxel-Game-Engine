use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Res, ResMut};
use bevy::render::renderer::RenderDevice;

use voxel_data::gpu_bvh::GpuBvh;
use voxel_data::grid::SubGridId;

/// Per-sub-grid GPU ray-hit counts from the previous frame.
///
/// Shared between the render world (writer) and the main world (reader) via
/// `Arc<Mutex<_>>`.
#[derive(Resource, Clone, Default)]
pub struct HitCountFeedback(pub Arc<Mutex<HashMap<(Entity, SubGridId), u32>>>);

/// Holds the `GpuBvh` produced by the previous frame's render so that we can
/// map its staging buffer this frame.
#[derive(Resource, Default)]
pub struct LastGpuBvh(pub Mutex<Option<GpuBvh>>);

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
