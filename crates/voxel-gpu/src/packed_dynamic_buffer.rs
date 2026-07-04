use std::{collections::BTreeMap, num::NonZero, ops::*};

use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};
use num::Integer;
use orderly_allocator::{Allocation, Allocator};
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

	fn allocation(&self) -> Allocation {
		Allocation {
			offset: self.offset,
			size: NonZero::new(self.size).expect("held buffers are never zero-sized"),
		}
	}
}

pub struct PackedDynamicBuffer {
	buffer: GpuBuffer,
	held_bytes: u32,
	alignment: u32,
	held_buffers: BTreeMap<u32, HeldBuffer>,
	allocator: Allocator,
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
			.field("alignment", &self.alignment)
			.field("held_buffers", &self.held_buffers)
			.field("allocator", &self.allocator)
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
			alignment,
			held_buffers: BTreeMap::new(),
			allocator: Allocator::new(buffer_size as u32),
			device: WgpuWrapper::new(raw_device.clone()),
			queue: WgpuWrapper::new(bevy::render::renderer::WgpuWrapper::clone(&**queue).into_inner()),
			usage,
			buffer_size,
			max_binding_size: raw_device.limits().max_storage_buffer_binding_size.min(u32::MAX as u64) as u32,
		})
	}

	pub fn alignment(&self) -> u32 {
		self.alignment
	}

	pub fn held_bytes(&self) -> u32 {
		self.held_bytes
	}

	fn grow_buffer(&mut self, min_size: u64) {
		let old_size = self.buffer_size;
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
		self
			.allocator
			.grow_capacity((new_size - old_size) as u32)
			.expect("PackedDynamicBuffer allocator capacity overflowed while growing");
	}

	fn allocate_buffer(&mut self, size: u32) -> Result<Allocation, &'static str> {
		if size == 0 {
			return Err("Buffer size can't be 0.");
		}
		if size > self.max_binding_size {
			return Err("Buffer max size hit!");
		}
		loop {
			if let Some(allocation) = self.allocator.alloc_with_align(size, self.alignment) {
				return Ok(allocation);
			}
			let required_total_size = self.buffer_size + (size as u64).next_multiple_of(self.alignment as u64);
			let next_size = self.buffer_size.max(required_total_size).next_power_of_two().next_multiple_of(self.alignment as u64);
			if next_size > self.max_binding_size as u64 {
				return Err("Buffer max size hit!");
			}
			self.grow_buffer(required_total_size);
		}
	}

	pub fn add_buffer(&mut self, data_buffer: &[u8]) -> Result<u32, &'static str> {
		let _zone = span!("PackedDynamicBuffer add_buffer");
		// tracy_client::plot!("packed dynamic upload bytes", data_buffer.len() as f64);
		let allocation = self.allocate_buffer(data_buffer.len() as u32)?;
		{
			let _write_zone = span!("wgpu queue write_buffer");
			self.queue.write_buffer(&self.buffer, allocation.offset as u64, data_buffer);
		}
		self.held_bytes += allocation.size();
		let id = allocation.offset / self.alignment;
		self.held_buffers.insert(id, HeldBuffer { offset: allocation.offset, size: allocation.size() });
		Ok(id)
	}

	pub fn remove_buffer(&mut self, id: u32) -> Result<(), &'static str> {
		if let Some(held_buffer) = self.held_buffers.remove(&id) {
			self.allocator.free(held_buffer.allocation());
			self.held_bytes -= held_buffer.size;
			Ok(())
		} else {
			Err("Could not find id.")
		}
	}

	/// If the new buffer does not fit the old buffer will still be removed
	pub fn replace_buffer(&mut self, id: u32, buffer: &[u8]) -> Result<u32, &'static str> {
		let _zone = span!("PackedDynamicBuffer replace_buffer");
		// tracy_client::plot!("packed dynamic upload bytes", buffer.len() as f64);
		let Some(held_buffer) = self.held_buffers.get_mut(&id) else {
			return Err("Could not find id.");
		};
		let Some(new_size) = u32::try_from(buffer.len()).ok() else {
			return Err("Buffer max size hit!");
		};
		if new_size == held_buffer.size {
			let _write_zone = span!("wgpu queue write_buffer");
			self.queue.write_buffer(&self.buffer, held_buffer.offset as u64, buffer);
			return Ok(id);
		}
		match self.allocator.try_reallocate(held_buffer.allocation(), new_size) {
			Ok(allocation) => {
				self.held_bytes = self.held_bytes - held_buffer.size + allocation.size();
				held_buffer.offset = allocation.offset;
				held_buffer.size = allocation.size();
				let _write_zone = span!("wgpu queue write_buffer");
				self.queue.write_buffer(&self.buffer, allocation.offset as u64, buffer);
				Ok(id)
			},
			Err(orderly_allocator::ReallocateError::InsufficientSpace { .. }) => {
				self.remove_buffer(id)?;
				self.add_buffer(buffer)
			},
			Err(orderly_allocator::ReallocateError::Invalid) => Err("Buffer size can't be 0."),
		}
	}

	pub fn held_buffer(&self, id: u32) -> Option<&HeldBuffer> {
		self.held_buffers.get(&id)
	}

	pub fn buffer(&self) -> &GpuBuffer {
		&self.buffer
	}
}
