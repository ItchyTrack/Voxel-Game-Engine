use std::{collections::HashMap, sync::{Arc, mpsc::{self, Receiver, Sender}}};

use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};
use orderly_allocator::{Allocation, Allocator};

pub type PackedBufferGroupBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuDevice = WgpuWrapper<wgpu::Device>;
type GpuQueue = WgpuWrapper<wgpu::Queue>;

const INITIAL_BUFFER_CAPACITY: u64 = 1 << 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct PackedBufferGroupId {
	buffer_index: u32,
	allocation: u64,
}

impl PackedBufferGroupId {
	pub fn buffer_index(self) -> u32 { self.buffer_index }
}

#[derive(Clone)]
pub struct PackedBufferGroupAllocation {
	owner: Arc<PackedBufferGroupAllocationOwner>,
}

struct PackedBufferGroupAllocationOwner {
	id: PackedBufferGroupId,
	cleanup: Sender<PackedBufferGroupId>,
}

impl std::fmt::Debug for PackedBufferGroupAllocation {
	fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		formatter.debug_struct("PackedBufferGroupAllocation").field("id", &self.owner.id).finish()
	}
}

impl PackedBufferGroupAllocation {
	pub fn id(&self) -> PackedBufferGroupId { self.owner.id }
}

impl Drop for PackedBufferGroupAllocationOwner {
	fn drop(&mut self) {
		let _ = self.cleanup.send(self.id);
	}
}

#[derive(Clone, Debug)]
pub struct PackedBufferGroupSlice {
	pub buffer: PackedBufferGroupBuffer,
	pub buffer_index: u32,
	pub offset: u32,
	pub size: u32,
}

struct HeldBuffer {
	allocation: Allocation,
}

struct PackedBuffer {
	buffer: PackedBufferGroupBuffer,
	capacity: u64,
	allocator: Allocator,
	held: HashMap<u64, HeldBuffer>,
}

impl PackedBuffer {
	fn new(device: &GpuDevice, capacity: u64, usage: wgpu::BufferUsages, label: &'static str) -> Self {
		let buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some(label),
			size: capacity,
			usage,
			mapped_at_creation: false,
		}));
		Self { buffer, capacity, allocator: Allocator::new(capacity as u32), held: HashMap::new() }
	}

	fn allocate(&mut self, size: u32, alignment: u32, allocation_id: u64) -> Option<Allocation> {
		let allocation = self.allocator.alloc_with_align(size, alignment)?;
		self.held.insert(allocation_id, HeldBuffer { allocation });
		Some(allocation)
	}
}

/// Long-lived, stable-offset GPU allocations split across storage-buffer-sized buffers.
///
/// Existing allocations are never moved during ordinary rendering. A packed buffer only changes
/// its backing GPU buffer when it grows; the complete old contents are copied first.
pub struct PackedBufferGroup {
	device: GpuDevice,
	queue: GpuQueue,
	usage: wgpu::BufferUsages,
	label: &'static str,
	alignment: u32,
	binding_limit: u64,
	buffers: Vec<PackedBuffer>,
	next_allocation: u64,
	held_bytes: u64,
	cleanup_tx: Sender<PackedBufferGroupId>,
	cleanup_rx: Receiver<PackedBufferGroupId>,
}

impl std::fmt::Debug for PackedBufferGroup {
	fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		formatter
			.debug_struct("PackedBufferGroup")
			.field("alignment", &self.alignment)
			.field("binding_limit", &self.binding_limit)
			.field("buffer_count", &self.buffers.len())
			.field("held_bytes", &self.held_bytes)
			.finish()
	}
}

impl PackedBufferGroup {
	pub fn new(
		render_device: &RenderDevice,
		render_queue: &RenderQueue,
		alignment: u32,
		usage: wgpu::BufferUsages,
		label: &'static str,
	) -> Result<Self, &'static str> {
		if alignment == 0 { return Err("Buffer alignment can't be 0."); }
		let raw_device = render_device.wgpu_device();
		let binding_limit = raw_device.limits().max_storage_buffer_binding_size.min(u32::MAX as u64);
		let alignment = alignment.next_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT as u32);
		let usage = usage | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC;
		let (cleanup_tx, cleanup_rx) = mpsc::channel();
		Ok(Self {
			device: WgpuWrapper::new(raw_device.clone()),
			queue: WgpuWrapper::new(WgpuWrapper::clone(&**render_queue).into_inner()),
			usage,
			label,
			alignment,
			binding_limit,
			buffers: Vec::new(),
			next_allocation: 1,
			held_bytes: 0,
			cleanup_tx,
			cleanup_rx,
		})
	}

	pub fn add_buffer(&mut self, data: Vec<u8>) -> Result<PackedBufferGroupAllocation, &'static str> {
		self.collect_garbage();
		let size = u32::try_from(data.len()).map_err(|_| "Buffer is larger than a storage-buffer binding.")?;
		if size == 0 { return Err("Buffer size can't be 0."); }
		if u64::from(size) > self.binding_limit { return Err("Buffer is larger than a storage-buffer binding."); }

		let allocation_id = self.next_allocation;
		self.next_allocation = self.next_allocation.wrapping_add(1).max(1);
		let mut placement = self.buffers.iter_mut().enumerate().find_map(|(index, buffer)| {
			buffer.allocate(size, self.alignment, allocation_id).map(|allocation| (index, allocation))
		});

		if placement.is_none() {
			placement = self.grow_last_buffer_and_allocate(size, allocation_id);
		}
		if placement.is_none() {
			let capacity = INITIAL_BUFFER_CAPACITY
				.max(u64::from(size).next_power_of_two())
				.min(self.binding_limit);
			self.buffers.push(PackedBuffer::new(&self.device, capacity, self.usage, self.label));
			let index = self.buffers.len() - 1;
			let allocation = self.buffers[index]
				.allocate(size, self.alignment, allocation_id)
				.expect("new packed buffer must fit its first allocation");
			placement = Some((index, allocation));
		}

		let (buffer_index, allocation) = placement.expect("packed buffer placement must exist");
		self.queue.write_buffer(&self.buffers[buffer_index].buffer, u64::from(allocation.offset), &data);
		self.held_bytes += u64::from(allocation.size());
		Ok(PackedBufferGroupAllocation {
			owner: Arc::new(PackedBufferGroupAllocationOwner {
				id: PackedBufferGroupId { buffer_index: buffer_index as u32, allocation: allocation_id },
				cleanup: self.cleanup_tx.clone(),
			}),
		})
	}

	pub fn slice(&self, id: PackedBufferGroupId) -> Option<PackedBufferGroupSlice> {
		let buffer = self.buffers.get(id.buffer_index as usize)?;
		let held = buffer.held.get(&id.allocation)?;
		Some(PackedBufferGroupSlice {
			buffer: buffer.buffer.clone(),
			buffer_index: id.buffer_index,
			offset: held.allocation.offset,
			size: held.allocation.size(),
		})
	}

	pub fn held_bytes(&self) -> u64 { self.held_bytes }
	pub fn buffer_count(&self) -> usize { self.buffers.len() }

	pub fn collect_garbage(&mut self) {
		while let Ok(id) = self.cleanup_rx.try_recv() {
			let Some(buffer) = self.buffers.get_mut(id.buffer_index as usize) else { continue };
			let Some(held) = buffer.held.remove(&id.allocation) else { continue };
			self.held_bytes -= u64::from(held.allocation.size());
			buffer.allocator.free(held.allocation);
		}
	}

	fn grow_last_buffer_and_allocate(&mut self, size: u32, allocation_id: u64) -> Option<(usize, Allocation)> {
		let index = self.buffers.len().checked_sub(1)?;
		let buffer = &mut self.buffers[index];
		if buffer.capacity >= self.binding_limit { return None; }

		let added_capacity = u64::from(size).next_multiple_of(u64::from(self.alignment));
		let new_capacity = buffer.capacity
			.saturating_add(added_capacity)
			.next_power_of_two()
			.min(self.binding_limit);
		if new_capacity <= buffer.capacity { return None; }

		let new_buffer = WgpuWrapper::new(self.device.create_buffer(&wgpu::BufferDescriptor {
			label: Some(self.label),
			size: new_capacity,
			usage: self.usage,
			mapped_at_creation: false,
		}));
		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
			label: Some("packed_buffer_group_grow"),
		});
		encoder.copy_buffer_to_buffer(&buffer.buffer, 0, &new_buffer, 0, buffer.capacity);
		self.queue.submit(std::iter::once(encoder.finish()));
		buffer.buffer = new_buffer;
		buffer.allocator.grow_capacity((new_capacity - buffer.capacity) as u32).ok()?;
		buffer.capacity = new_capacity;
		buffer.allocate(size, self.alignment, allocation_id).map(|allocation| (index, allocation))
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn allocation_cleanup_waits_for_last_lease() {
		let (cleanup, cleaned) = mpsc::channel();
		let id = PackedBufferGroupId { buffer_index: 2, allocation: 7 };
		let allocation = PackedBufferGroupAllocation {
			owner: Arc::new(PackedBufferGroupAllocationOwner { id, cleanup }),
		};
		let render_lease = allocation.clone();

		drop(allocation);
		assert!(cleaned.try_recv().is_err());
		drop(render_lease);
		assert_eq!(cleaned.try_recv(), Ok(id));
	}
}
