use rustc_hash::FxHashMap;
use std::sync::{Arc, Mutex};

use bevy::ecs::component::Component;
use bevy::ecs::system::{Query, Res};
use bevy::prelude::Entity;
use bevy::render::renderer::RenderDevice;

use crate::gpu_bvh::GpuBvh;
use crate::gpu_timing::{FrameTimings, GpuPassTimer, RenderCpuSample};
use crate::incoming_ray_directions::IncomingRayDirections;

#[derive(Component, Default)]
pub struct DirectionFeedback {
	directions: FxHashMap<Entity, IncomingRayDirections>,
	rendered: Option<GpuBvh>,
	pub(crate) timer: Option<GpuPassTimer>,
	pub(crate) gpu_bvh_cache: crate::gpu_bvh::GpuBvhCache,
	/// Benchmark tag of the frame currently being extracted and rendered.
	pub(crate) frame_tag: u32,
}

impl DirectionFeedback {
	pub fn directions_for(&self, entity: Entity) -> IncomingRayDirections {
		self.directions.get(&entity).copied().unwrap_or_default()
	}

	pub fn push_rendered(&mut self, gpu_bvh: GpuBvh) {
		assert!(self.rendered.is_none(), "direction feedback from the previous render was not consumed");
		self.rendered = Some(gpu_bvh);
	}
}

#[derive(bevy::ecs::resource::Resource, Clone, Default)]
pub struct RenderStats {
	pub inner: Arc<Mutex<RenderStatsData>>,
	pub timing: FrameTimings,
}

#[derive(Default, Clone, Copy, Debug)]
pub struct RenderStatsData {
	pub bvh_bytes: u64,
	pub bvh_leaf_bytes: u64,
}

pub fn read_back_direction_masks(
	render_device: Res<RenderDevice>,
	render_stats: Res<RenderStats>,
	mut view_feedback: Query<&mut DirectionFeedback>,
) {
	let timing_enabled = render_stats.timing.enabled();
	for mut feedback in &mut view_feedback {
		let start = std::time::Instant::now();
		feedback.frame_tag = render_stats.timing.tag();
		if let Some(timer) = feedback.timer.as_mut() {
			timer.collect(render_device.wgpu_device(), &render_stats.timing);
		}
		read_back_view(&render_device, &mut feedback);
		if timing_enabled {
			render_stats.timing.push_cpu(RenderCpuSample {
				tag: feedback.frame_tag,
				readback_ms: start.elapsed().as_secs_f32() * 1e3,
				extract_ms: f32::NAN,
			});
		}
	}
}

fn read_back_view(render_device: &RenderDevice, feedback: &mut DirectionFeedback) {
	{
		let Some(gpu_bvh) = feedback.rendered.take() else { return };
		gpu_bvh.item_direction_mask_staging_buffer.slice(..).map_async(
			wgpu::MapMode::Read,
			|result| result.expect("failed to map voxel direction feedback"),
		);
		render_device.wgpu_device().poll(wgpu::PollType::Wait {
			submission_index: None,
			timeout: None,
		}).expect("failed while waiting for voxel direction feedback");

		let staging = gpu_bvh.item_direction_mask_staging_buffer.slice(..);
		feedback.directions = {
			let view = staging.get_mapped_range();
			let words: &[u32] = bytemuck::cast_slice(&view);
			gpu_bvh.item_ids.iter().enumerate().map(|(item_index, id)| {
				let word = words[item_index / 4];
				let shift = (item_index % 4) * 8;
				(*id, IncomingRayDirections::from_bits_truncate((word >> shift) as u8))
			}).collect()
		};
		gpu_bvh.item_direction_mask_staging_buffer.unmap();
	}
}
