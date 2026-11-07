//! Per-pass GPU timestamps and render-thread CPU timings for the benchmark harness.
//! Disabled unless [`FrameTimings::set_enabled`] is called, so normal runs do no query work.

use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

use bevy::render::renderer::WgpuWrapper;

pub const NO_TAG: u32 = u32::MAX;

const QUERY_COUNT: u32 = 6;
const QUERY_BYTES: u64 = QUERY_COUNT as u64 * size_of::<u64>() as u64;
const SLOT_COUNT: usize = 8;

const SLOT_FREE: u8 = 0;
const SLOT_RECORDED: u8 = 1;
const SLOT_MAPPING: u8 = 2;
const SLOT_READY: u8 = 3;

#[derive(Clone, Copy, Debug)]
pub struct GpuPassSample {
	pub tag: u32,
	pub beam_ms: f32,
	pub ray_ms: f32,
	pub color_ms: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct RenderCpuSample {
	pub tag: u32,
	pub readback_ms: f32,
	pub extract_ms: f32,
}

/// Shared between the main world (benchmark driver) and the render world.
#[derive(Clone, Default)]
pub struct FrameTimings {
	enabled: Arc<AtomicBool>,
	tag: Arc<AtomicU32>,
	gpu: Arc<Mutex<Vec<GpuPassSample>>>,
	cpu: Arc<Mutex<Vec<RenderCpuSample>>>,
}

impl FrameTimings {
	pub fn set_enabled(&self, enabled: bool) { self.enabled.store(enabled, Ordering::Relaxed); }
	pub fn enabled(&self) -> bool { self.enabled.load(Ordering::Relaxed) }
	/// Tag the frame the main world is currently producing; samples for it carry this value.
	pub fn set_tag(&self, tag: u32) { self.tag.store(tag, Ordering::Relaxed); }
	pub fn tag(&self) -> u32 { self.tag.load(Ordering::Relaxed) }
	pub fn push_cpu(&self, sample: RenderCpuSample) { self.cpu.lock().unwrap().push(sample); }
	pub fn drain_gpu(&self) -> Vec<GpuPassSample> { std::mem::take(&mut *self.gpu.lock().unwrap()) }
	pub fn drain_cpu(&self) -> Vec<RenderCpuSample> { std::mem::take(&mut *self.cpu.lock().unwrap()) }
}

struct Slot {
	buffer: WgpuWrapper<wgpu::Buffer>,
	state: Arc<AtomicU8>,
	tag: u32,
}

pub struct GpuPassTimer {
	query_set: WgpuWrapper<wgpu::QuerySet>,
	resolve: WgpuWrapper<wgpu::Buffer>,
	slots: Vec<Slot>,
	period_ns: f32,
}

impl GpuPassTimer {
	pub fn new(device: &wgpu::Device, period_ns: f32) -> Option<Self> {
		if !device.features().contains(wgpu::Features::TIMESTAMP_QUERY) { return None; }
		let query_set = WgpuWrapper::new(device.create_query_set(&wgpu::QuerySetDescriptor {
			label: Some("voxel_ray_pass_timestamps"),
			ty: wgpu::QueryType::Timestamp,
			count: QUERY_COUNT,
		}));
		let resolve = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("voxel_ray_timestamp_resolve"),
			size: QUERY_BYTES,
			usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
			mapped_at_creation: false,
		}));
		let slots = (0..SLOT_COUNT).map(|_| Slot {
			buffer: WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
				label: Some("voxel_ray_timestamp_readback"),
				size: QUERY_BYTES,
				usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
				mapped_at_creation: false,
			})),
			state: Arc::new(AtomicU8::new(SLOT_FREE)),
			tag: NO_TAG,
		}).collect();
		Some(Self { query_set, resolve, slots, period_ns })
	}

	pub fn free_slot(&self) -> Option<usize> {
		self.slots.iter().position(|slot| slot.state.load(Ordering::Acquire) == SLOT_FREE)
	}

	pub fn compute_writes(&self, pass: u32) -> wgpu::ComputePassTimestampWrites<'_> {
		wgpu::ComputePassTimestampWrites {
			query_set: &self.query_set,
			beginning_of_pass_write_index: Some(pass * 2),
			end_of_pass_write_index: Some(pass * 2 + 1),
		}
	}

	pub fn render_writes(&self, pass: u32) -> wgpu::RenderPassTimestampWrites<'_> {
		wgpu::RenderPassTimestampWrites {
			query_set: &self.query_set,
			beginning_of_pass_write_index: Some(pass * 2),
			end_of_pass_write_index: Some(pass * 2 + 1),
		}
	}

	pub fn resolve(&mut self, encoder: &mut wgpu::CommandEncoder, slot: usize, tag: u32) {
		encoder.resolve_query_set(&self.query_set, 0..QUERY_COUNT, &self.resolve, 0);
		encoder.copy_buffer_to_buffer(&self.resolve, 0, &self.slots[slot].buffer, 0, QUERY_BYTES);
		self.slots[slot].tag = tag;
		self.slots[slot].state.store(SLOT_RECORDED, Ordering::Release);
	}

	/// Must run after the frame that recorded a slot has been submitted.
	pub fn collect(&mut self, device: &wgpu::Device, timings: &FrameTimings) {
		for slot in &self.slots {
			if slot.state.load(Ordering::Acquire) == SLOT_RECORDED {
				slot.state.store(SLOT_MAPPING, Ordering::Release);
				let state = slot.state.clone();
				slot.buffer.slice(..).map_async(wgpu::MapMode::Read, move |result| {
					result.expect("failed to map voxel timestamp readback");
					state.store(SLOT_READY, Ordering::Release);
				});
			}
		}
		let _ = device.poll(wgpu::PollType::Poll);
		for slot in &self.slots {
			if slot.state.load(Ordering::Acquire) != SLOT_READY { continue; }
			{
				let view = slot.buffer.slice(..).get_mapped_range();
				let ticks: &[u64] = bytemuck::cast_slice(&view);
				let ms = |pass: usize| ticks[pass * 2 + 1].saturating_sub(ticks[pass * 2]) as f32 * self.period_ns / 1e6;
				timings.gpu.lock().unwrap().push(GpuPassSample { tag: slot.tag, beam_ms: ms(0), ray_ms: ms(1), color_ms: ms(2) });
			}
			slot.buffer.unmap();
			slot.state.store(SLOT_FREE, Ordering::Release);
		}
	}
}
