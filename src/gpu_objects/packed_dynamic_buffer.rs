use std::{collections::BTreeMap, ops::*};

use num::Integer;
use wgpu::CommandEncoderDescriptor;

pub struct HeldBuffer {
	offset: u32,
	size: u32,
}

impl HeldBuffer {
	pub fn offset(&self) -> u32 { self.offset }
	pub fn size(&self) -> u32 { self.size }
}

pub struct PackedDynamicBuffer {
	buffer: wgpu::Buffer,
	held_bytes: u32,
	held_bytes_alignment: u32,
	alignment: u32,
	held_buffers: BTreeMap<u32, HeldBuffer>,
}

impl PackedDynamicBuffer {
	pub fn new(device: &wgpu::Device, alignment: u32, usage: wgpu::BufferUsages) -> Result<Self, &'static str> {
		let alignment = alignment.lcm(&(wgpu::COPY_BUFFER_ALIGNMENT as u32));
		let size = (1u32.shl(20u32)).next_multiple_of(alignment as u32);
		let buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("PackedBuffer"),
			size: size as u64,
			usage: usage | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
			mapped_at_creation: false,
		});
		Ok(Self {
			buffer,
			held_bytes: 0,
			held_bytes_alignment: 0,
			alignment: alignment,
			held_buffers: BTreeMap::new(),
		})
	}

	pub fn alignment(&self) -> u32 {
		self.alignment
	}

	pub fn held_bytes(&self) -> u32 {
		self.held_bytes
	}

	pub fn add_buffer(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, data_buffer: &[u8]) -> Result<u32, &'static str> {
		if data_buffer.len() == 0 {
			return Err("Buffer size can't be 0.");
		}
		if data_buffer.len() as u32 > self.buffer.size() as u32 - self.held_bytes_alignment {
			if self.buffer.size().shl(1u32).next_multiple_of(self.alignment as u64) > device.limits().max_storage_buffer_binding_size {
				return Err("Buffer max size hit!");
			}
			let new_buffer = device.create_buffer(&wgpu::BufferDescriptor {
				label: Some("PackedBuffer"),
				size: self.buffer.size().shl(1u32).next_multiple_of(self.alignment as u64),
				usage: self.buffer.usage(),
				mapped_at_creation: false,
			});
			let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor {
				label: Some("e"),
			});
			encoder.copy_buffer_to_buffer(&self.buffer, 0, &new_buffer, 0, self.buffer.size());
			queue.submit(std::iter::once(encoder.finish()));
			self.buffer = new_buffer;
		}
		let mut placement_location = 0;
		loop {
			let range = self.held_buffers.range(
				(placement_location / self.alignment as u32) as u32..
				(placement_location + data_buffer.len() as u32).div_ceil(self.alignment as u32) as u32
			);
			if let Some((_, held_buffer)) = range.last() {
				placement_location = held_buffer.offset + held_buffer.size.next_multiple_of(self.alignment as u32);
				assert!(placement_location.is_multiple_of(self.alignment as u32));
				if (placement_location + data_buffer.len() as u32) > (self.buffer.size() as u32) {
					if self.buffer.size().shl(1u32).next_multiple_of(self.alignment as u64) > device.limits().max_storage_buffer_binding_size {
				return Err("Buffer max size hit!");
			}
					let new_buffer = device.create_buffer(&wgpu::BufferDescriptor {
						label: Some("PackedBuffer"),
						size: self.buffer.size().shl(1u32).next_multiple_of(self.alignment as u64),
						usage: self.buffer.usage(),
						mapped_at_creation: false,
					});
					let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor {
						label: Some("e"),
					});
					encoder.copy_buffer_to_buffer(&self.buffer, 0, &new_buffer, 0, self.buffer.size());
					queue.submit(std::iter::once(encoder.finish()));
					placement_location = self.buffer.size() as u32;
					self.buffer = new_buffer;
					queue.write_buffer(&self.buffer, placement_location as u64, data_buffer);
					self.held_bytes += data_buffer.len() as u32;
					self.held_bytes_alignment += (data_buffer.len() as u32).next_multiple_of(self.alignment as u32);
					let id = placement_location / self.alignment as u32;
					self.held_buffers.insert(id, HeldBuffer { offset: placement_location, size: data_buffer.len() as u32 });
					return Ok(id);
				}
			} else {
				break;
			}
		}
		queue.write_buffer(&self.buffer, placement_location as u64, data_buffer);
		self.held_bytes += data_buffer.len() as u32;
		self.held_bytes_alignment += (data_buffer.len() as u32).next_multiple_of(self.alignment as u32);
		let id = placement_location / self.alignment as u32;
		self.held_buffers.insert(id, HeldBuffer { offset: placement_location, size: data_buffer.len() as u32 });
		Ok(id)
	}

	pub fn remove_buffer(&mut self, id: u32) -> Result<(), &'static str> {
		if let Some(held_buffer) = self.held_buffers.remove(&id) {
			self.held_bytes -= held_buffer.size;
			self.held_bytes_alignment -= held_buffer.size.next_multiple_of(self.alignment as u32);
			Ok(())
		} else {
			Err("Could not find id.")
		}
	}

	// If the new buffer does not fit the old buffer will still be removed
	pub fn replace_buffer(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, id: u32, buffer: &[u8]) -> Result<u32, &'static str> {
		if let Some(held_buffer) = self.held_buffers.get_mut(&id) {
			if held_buffer.size == buffer.len() as u32 {
				self.held_bytes -= held_buffer.size;
				self.held_bytes += buffer.len() as u32;
				self.held_bytes_alignment -= held_buffer.size.next_multiple_of(self.alignment as u32);
				self.held_bytes_alignment += (buffer.len() as u32).next_multiple_of(self.alignment as u32);
				held_buffer.size = buffer.len() as u32;
				queue.write_buffer(&self.buffer, held_buffer.offset as u64, buffer);
				Ok(id)
			} else {
				self.remove_buffer(id)?;
				self.add_buffer(device, queue, buffer)
			}
		} else {
			Err("Could not find id.")
		}
	}

	pub fn get_held_buffer(&self, id: u32) -> Option<&HeldBuffer> {
		self.held_buffers.get(&id)
	}

	pub fn get_buffer(&self) -> &wgpu::Buffer {
		&self.buffer
	}
}
