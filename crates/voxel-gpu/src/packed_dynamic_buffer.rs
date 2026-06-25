use std::{collections::BTreeMap, ops::*};

use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};
use num::Integer;
use tracy_client::span;

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuDevice = WgpuWrapper<wgpu::Device>;
type GpuQueue = WgpuWrapper<wgpu::Queue>;

#[derive(Debug)]
pub struct HeldBuffer {
	offset: u32,
	size: u32,
}

impl HeldBuffer {
	pub fn offset(&self) -> u32 { self.offset }
	pub fn size(&self) -> u32 { self.size }
}

pub struct PackedDynamicBuffer {
	buffer: GpuBuffer,
	held_bytes: u32,
	held_bytes_alignment: u32,
	alignment: u32,
	held_buffers: BTreeMap<u32, HeldBuffer>,
	device: GpuDevice,
	queue: GpuQueue,
	usage: wgpu::BufferUsages,
	buffer_size: u64,
	max_binding_size: u32,
}

impl std::fmt::Debug for PackedDynamicBuffer {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		f.debug_struct("PackedDynamicBuffer")
			.field("buffer", &self.buffer)
			.field("held_bytes", &self.held_bytes)
			.field("held_bytes_alignment", &self.held_bytes_alignment)
			.field("alignment", &self.alignment)
			.field("held_buffers", &self.held_buffers)
			.field("device", &self.device)
			.field("queue", &self.queue)
			.finish()
	}
}

impl PackedDynamicBuffer {
	pub fn new(device: &RenderDevice, queue: &RenderQueue, alignment: u32, usage: wgpu::BufferUsages) -> Result<Self, &'static str> {
		let alignment = alignment.lcm(&(wgpu::COPY_BUFFER_ALIGNMENT as u32));
		let buffer_size = (1u32.shl(20u32)).next_multiple_of(alignment) as u64;
		let raw_device = device.wgpu_device();
		let buffer = WgpuWrapper::new(raw_device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("packed_dynamic_buffer"),
			size: buffer_size,
			usage,
			mapped_at_creation: false,
		}));
		Ok(Self {
			buffer,
			held_bytes: 0,
			held_bytes_alignment: 0,
			alignment,
			held_buffers: BTreeMap::new(),
			device: WgpuWrapper::new(raw_device.clone()),
			queue: WgpuWrapper::new(bevy::render::renderer::WgpuWrapper::clone(&**queue).into_inner()),
			usage,
			buffer_size,
			max_binding_size: raw_device.limits().max_storage_buffer_binding_size,
		})
	}

	pub fn alignment(&self) -> u32 {
		self.alignment
	}

	pub fn held_bytes(&self) -> u32 {
		self.held_bytes
	}

	fn grow_buffer(&mut self, min_size: u64) {
		let new_size = self.buffer_size.max(min_size).next_power_of_two().next_multiple_of(self.alignment as u64);
		let new_buffer = WgpuWrapper::new(self.device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("PackedBuffer"),
			size: new_size,
			usage: self.usage,
			mapped_at_creation: false,
		}));
		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("packed_dynamic_copy") });
		encoder.copy_buffer_to_buffer(&self.buffer, 0, &new_buffer, 0, self.buffer_size);
		self.queue.submit(std::iter::once(encoder.finish()));
		self.buffer = new_buffer;
		self.buffer_size = new_size;
	}

	pub fn add_buffer(&mut self, data_buffer: &[u8]) -> Result<u32, &'static str> {
		let _zone = span!("PackedDynamicBuffer add_buffer");
		// tracy_client::plot!("packed dynamic upload bytes", data_buffer.len() as f64);
		if data_buffer.is_empty() {
			return Err("Buffer size can't be 0.");
		}
		if data_buffer.len() as u32 > self.buffer_size as u32 - self.held_bytes_alignment {
			let next_size = self.buffer_size.shl(1u32).next_multiple_of(self.alignment as u64);
			if next_size > self.max_binding_size as u64 {
				return Err("Buffer max size hit!");
			}
			self.grow_buffer(next_size);
		}
		let mut placement_location = 0;
		loop {
			let range = self.held_buffers.range(
				(placement_location / self.alignment) as u32..
				(placement_location + data_buffer.len() as u32).div_ceil(self.alignment) as u32
			);
			if let Some((_, held_buffer)) = range.last() {
				placement_location = held_buffer.offset + held_buffer.size.next_multiple_of(self.alignment);
				assert!(placement_location.is_multiple_of(self.alignment));
				if (placement_location + data_buffer.len() as u32) > self.buffer_size as u32 {
					let next_size = self.buffer_size.shl(1u32).next_multiple_of(self.alignment as u64);
					if next_size > self.max_binding_size as u64 {
						return Err("Buffer max size hit!");
					}
					self.grow_buffer(next_size);
					placement_location = self.buffer_size as u32 / 2;
					{
						let _write_zone = span!("wgpu queue write_buffer");
						self.queue.write_buffer(&self.buffer, placement_location as u64, data_buffer);
					}
					self.held_bytes += data_buffer.len() as u32;
					self.held_bytes_alignment += (data_buffer.len() as u32).next_multiple_of(self.alignment);
					let id = placement_location / self.alignment;
					self.held_buffers.insert(id, HeldBuffer { offset: placement_location, size: data_buffer.len() as u32 });
					return Ok(id);
				}
			} else {
				break;
			}
		}
		{
			let _write_zone = span!("wgpu queue write_buffer");
			self.queue.write_buffer(&self.buffer, placement_location as u64, data_buffer);
		}
		self.held_bytes += data_buffer.len() as u32;
		self.held_bytes_alignment += (data_buffer.len() as u32).next_multiple_of(self.alignment);
		let id = placement_location / self.alignment;
		self.held_buffers.insert(id, HeldBuffer { offset: placement_location, size: data_buffer.len() as u32 });
		Ok(id)
	}

	pub fn remove_buffer(&mut self, id: u32) -> Result<(), &'static str> {
		if let Some(held_buffer) = self.held_buffers.remove(&id) {
			self.held_bytes -= held_buffer.size;
			self.held_bytes_alignment -= held_buffer.size.next_multiple_of(self.alignment);
			Ok(())
		} else {
			Err("Could not find id.")
		}
	}

	/// If the new buffer does not fit the old buffer will still be removed
	pub fn replace_buffer(&mut self, id: u32, buffer: &[u8]) -> Result<u32, &'static str> {
		let _zone = span!("PackedDynamicBuffer replace_buffer");
		// tracy_client::plot!("packed dynamic upload bytes", buffer.len() as f64);
		if let Some(held_buffer) = self.held_buffers.get_mut(&id) {
			if held_buffer.size == buffer.len() as u32 {
				self.held_bytes -= held_buffer.size;
				self.held_bytes += buffer.len() as u32;
				self.held_bytes_alignment -= held_buffer.size.next_multiple_of(self.alignment);
				self.held_bytes_alignment += (buffer.len() as u32).next_multiple_of(self.alignment);
				held_buffer.size = buffer.len() as u32;
				{
					let _write_zone = span!("wgpu queue write_buffer");
					self.queue.write_buffer(&self.buffer, held_buffer.offset as u64, buffer);
				}
				Ok(id)
			} else {
				self.remove_buffer(id)?;
				self.add_buffer(buffer)
			}
		} else {
			Err("Could not find id.")
		}
	}

	pub fn held_buffer(&self, id: u32) -> Option<&HeldBuffer> {
		self.held_buffers.get(&id)
	}

	pub fn buffer(&self) -> &GpuBuffer {
		&self.buffer
	}
}
