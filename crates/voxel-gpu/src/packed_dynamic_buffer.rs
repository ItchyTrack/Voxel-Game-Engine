use std::{
	collections::HashMap,
	num::NonZero,
	ops::*,
	sync::{
		atomic::{AtomicUsize, Ordering},
		mpsc::{self, Receiver, Sender},
		Mutex,
	},
};

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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct AllocationId(u32);

pub struct PackedBufferAllocation {
	id: AllocationId,
	cleanup: Sender<u32>,
}

impl std::fmt::Debug for PackedBufferAllocation {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		f.debug_struct("PackedBufferAllocation").field("id", &self.id).finish()
	}
}

impl PackedBufferAllocation {
	pub fn id(&self) -> AllocationId { self.id }

	fn into_id(self) -> AllocationId {
		let allocation = std::mem::ManuallyDrop::new(self);
		let id = allocation.id;
		unsafe {
			drop(std::ptr::read(std::ptr::addr_of!(allocation.cleanup)));
		}
		id
	}
}

impl Drop for PackedBufferAllocation {
	fn drop(&mut self) {
		let _ = self.cleanup.send(self.id.0);
	}
}

pub struct PackedDynamicBuffer {
	buffer: GpuBuffer,
	held_bytes: u32,
	alignment: u32,
	held_buffers: HashMap<u32, HeldBuffer>,
	allocator: Allocator,
	device: GpuDevice,
	queue: GpuQueue,
	usage: wgpu::BufferUsages,
	buffer_size: u64,
	max_binding_size: u32,
	cleanup_tx: Sender<u32>,
	cleanup_rx: Receiver<u32>,
	pending_uploads: Mutex<Vec<(u32, Vec<u8>)>>,
	pending_upload_count: AtomicUsize,
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
		let (cleanup_tx, cleanup_rx) = mpsc::channel();
		Ok(Self {
			buffer,
			held_bytes: 0,
			alignment,
			held_buffers: HashMap::new(),
			allocator: Allocator::new(buffer_size as u32),
			device: WgpuWrapper::new(raw_device.clone()),
			queue: WgpuWrapper::new(bevy::render::renderer::WgpuWrapper::clone(&**queue).into_inner()),
			usage,
			buffer_size,
			max_binding_size: raw_device.limits().max_storage_buffer_binding_size.min(u32::MAX as u64) as u32,
			cleanup_tx,
			cleanup_rx,
			pending_uploads: Mutex::new(Vec::new()),
			pending_upload_count: AtomicUsize::new(0),
		})
	}

	pub fn alignment(&self) -> u32 {
		self.alignment
	}

	pub fn held_bytes(&self) -> u32 {
		self.complete_uploads();
		self.held_bytes
	}

	fn grow_buffer(&mut self, min_size: u64) {
		self.complete_uploads();
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

	pub fn add_buffer(&mut self, data_buffer: Vec<u8>) -> Result<PackedBufferAllocation, &'static str> {
		self.collect_garbage();
		let id = self.add_buffer_raw(data_buffer)?;
		Ok(PackedBufferAllocation { id: AllocationId(id), cleanup: self.cleanup_tx.clone() })
	}

	pub fn remove_buffer(&mut self, allocation: PackedBufferAllocation) -> Result<(), &'static str> {
		self.collect_garbage();
		self.remove_buffer_raw(allocation.into_id().0)
	}

	/// If the new buffer does not fit the old buffer will still be removed.
	pub fn replace_buffer(&mut self, allocation: PackedBufferAllocation, buffer: Vec<u8>) -> Result<PackedBufferAllocation, &'static str> {
		self.collect_garbage();
		let id = self.replace_buffer_raw(allocation.into_id().0, buffer)?;
		Ok(PackedBufferAllocation { id: AllocationId(id), cleanup: self.cleanup_tx.clone() })
	}

	pub fn collect_garbage(&mut self) {
		while let Ok(id) = self.cleanup_rx.try_recv() {
			let _ = self.remove_buffer_raw(id);
		}
	}

	fn add_buffer_raw(&mut self, data_buffer: Vec<u8>) -> Result<u32, &'static str> {
		let _zone = span!("PackedDynamicBuffer add_buffer");
		// tracy_client::plot!("packed dynamic upload bytes", data_buffer.len() as f64);
		let allocation = self.allocate_buffer(data_buffer.len() as u32)?;
		self.queue_upload(allocation.offset, data_buffer);
		self.held_bytes += allocation.size();
		let id = allocation.offset / self.alignment;
		self.held_buffers.insert(id, HeldBuffer { offset: allocation.offset, size: allocation.size() });
		Ok(id)
	}

	fn remove_buffer_raw(&mut self, id: u32) -> Result<(), &'static str> {
		if let Some(held_buffer) = self.held_buffers.remove(&id) {
			self.allocator.free(held_buffer.allocation());
			self.held_bytes -= held_buffer.size;
			Ok(())
		} else {
			Err("Could not find id.")
		}
	}

	fn replace_buffer_raw(&mut self, id: u32, buffer: Vec<u8>) -> Result<u32, &'static str> {
		let _zone = span!("PackedDynamicBuffer replace_buffer");
		// tracy_client::plot!("packed dynamic upload bytes", buffer.len() as f64);
		let Some(held_buffer) = self.held_buffers.get_mut(&id) else {
			return Err("Could not find id.");
		};
		let Some(new_size) = u32::try_from(buffer.len()).ok() else {
			return Err("Buffer max size hit!");
		};
		if new_size == held_buffer.size {
			let offset = held_buffer.offset;
			self.queue_upload(offset, buffer);
			return Ok(id);
		}
		match self.allocator.try_reallocate(held_buffer.allocation(), new_size) {
			Ok(allocation) => {
				self.held_bytes = self.held_bytes - held_buffer.size + allocation.size();
				held_buffer.offset = allocation.offset;
				held_buffer.size = allocation.size();
				self.queue_upload(allocation.offset, buffer);
				Ok(id)
			},
			Err(orderly_allocator::ReallocateError::InsufficientSpace { .. }) => {
				self.remove_buffer_raw(id)?;
				self.add_buffer_raw(buffer)
			},
			Err(orderly_allocator::ReallocateError::Invalid) => Err("Buffer size can't be 0."),
		}
	}

	fn queue_upload(&mut self, offset: u32, data: Vec<u8>) {
		self.pending_uploads.get_mut().unwrap().push((offset, data));
		self.pending_upload_count.fetch_add(1, Ordering::Release);
	}

	fn complete_uploads(&self) {
		if self.pending_upload_count.load(Ordering::Acquire) == 0 { return; }
		let mut pending = self.pending_uploads.lock().unwrap();
		let upload_count = pending.len();
		let _write_zone = span!("PackedDynamicBuffer complete uploads");
		for (offset, data) in pending.drain(..) {
			self.queue.write_buffer(&self.buffer, offset as u64, &data);
		}
		self.pending_upload_count.fetch_sub(upload_count, Ordering::AcqRel);
	}

	pub fn held_buffer(&self, id: AllocationId) -> Option<&HeldBuffer> {
		self.complete_uploads();
		self.held_buffers.get(&id.0)
	}

	pub fn buffer(&self) -> &GpuBuffer {
		self.complete_uploads();
		&self.buffer
	}
}
