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
	latest: Option<FeedbackFrame>,
	rendered: Option<GpuBvh>,
	pub(crate) timer: Option<GpuPassTimer>,
	/// Benchmark tag of the frame currently being extracted and rendered.
	pub(crate) frame_tag: u32,
}

impl DirectionFeedback {
	/// Ray directions that hit each tile in the last frame read back.
	pub fn latest(&self) -> Option<&FeedbackFrame> { self.latest.as_ref() }

	pub fn push_rendered(&mut self, gpu_bvh: GpuBvh) {
		assert!(self.rendered.is_none(), "direction feedback from the previous render was not consumed");
		self.rendered = Some(gpu_bvh);
	}
}

pub struct FeedbackFrame {
	/// Version of the tile table the slots refer to.
	pub version: u64,
	/// Tiles touched by at least one ray, by table slot.
	pub slot_directions: Vec<(u32, IncomingRayDirections)>,
	entities: Vec<Entity>,
}

impl FeedbackFrame {
	pub fn entity_directions(&self) -> impl Iterator<Item = (Entity, IncomingRayDirections)> + '_ {
		self.entities.iter().zip(&self.slot_directions).map(|(&entity, &(_, directions))| (entity, directions))
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
	render_stats.timing.set_extracted_tag(render_stats.timing.tag());
	for mut feedback in &mut view_feedback {
		let start = std::time::Instant::now();
		feedback.frame_tag = render_stats.timing.tag();
		if let Some(timer) = feedback.timer.as_mut() {
			timer.collect(render_device.wgpu_device(), &render_stats.timing);
		}
		read_back_view(&render_device, &mut feedback);
		if timing_enabled {
			render_stats.timing.push_cpu(RenderCpuSample {
				readback_ms: start.elapsed().as_secs_f32() * 1e3,
				..RenderCpuSample::new(feedback.frame_tag)
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
		feedback.latest = {
			let view = staging.get_mapped_range();
			let words: &[u32] = bytemuck::cast_slice(&view);
			let mut slot_directions = Vec::new();
			let mut entities = Vec::new();
			for (item_index, (&slot, &entity)) in gpu_bvh.item_slots.iter().zip(&gpu_bvh.item_entities).enumerate() {
				let directions = IncomingRayDirections::from_bits_truncate((words[item_index / 4] >> ((item_index % 4) * 8)) as u8);
				if directions.is_empty() { continue; }
				slot_directions.push((slot, directions));
				entities.push(entity);
			}
			Some(FeedbackFrame { version: gpu_bvh.table_version, slot_directions, entities })
		};
		gpu_bvh.item_direction_mask_staging_buffer.unmap();
	}
}
