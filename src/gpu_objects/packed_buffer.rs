use std::collections::BTreeMap;

use clone_cell::clone::PureClone;
use num::Integer;

pub struct HeldBuffer {
	offset: u32,
	size: u32,
}

impl HeldBuffer {
	pub fn offset(&self) -> u32 { self.offset }
	pub fn size(&self) -> u32 { self.size }
}

pub struct PackedBuffer {
	buffer: wgpu::Buffer,
	held_bytes: u32,
	held_bytes_alignment: u32,
	alignment: u32,
	held_buffers: BTreeMap<u32, HeldBuffer>,
}

impl PackedBuffer {
	pub fn new(device: &wgpu::Device, size: u32, alignment: u32, usage: wgpu::BufferUsages) -> Result<Self, &'static str> {
		let alignment = alignment.lcm(&(wgpu::COPY_BUFFER_ALIGNMENT as u32));
		let size = size.next_multiple_of(alignment as u32);
		let buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("PackedBuffer"),
			size: size as u64,
			usage: usage | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		});
		Ok(Self {
			buffer,
			held_bytes: 0,
			held_bytes_alignment: 0,
			alignment: alignment,
			held_buffers: BTreeMap::new()
		})
	}

	pub fn add_buffer(&mut self, queue: &wgpu::Queue, buffer: &[u8]) -> Result<u32, &'static str> {
		if buffer.len() == 0 {
			return Err("Buffer size can't be 0.");
		}
		if buffer.len() as u32 > self.buffer.size() as u32 - self.held_bytes_alignment {
			return Err("Not enough free space.");
		}
		let mut placement_location = 0;
		loop {
			let range = self.held_buffers.range(
				(placement_location / self.alignment as u32) as u32..
				(placement_location + buffer.len() as u32).div_ceil(self.alignment as u32) as u32
			);
			if let Some((_, held_buffer)) = range.last() {
				placement_location = held_buffer.offset + held_buffer.size.next_multiple_of(self.alignment as u32);
				assert!(placement_location.is_multiple_of(self.alignment as u32));
				if (placement_location + buffer.len() as u32) > (self.buffer.size() as u32) {
					return Err("Not enough free space in single allocation.");
				}
			} else {
				break;
			}
		}
		queue.write_buffer(&self.buffer, placement_location as u64, buffer);
		self.held_bytes += buffer.len() as u32;
		self.held_bytes_alignment += (buffer.len() as u32).next_multiple_of(self.alignment as u32);
		let id = (placement_location / self.alignment as u32) as u32;
		self.held_buffers.insert(id, HeldBuffer { offset: placement_location, size: buffer.len() as u32 });
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
	pub fn replace_buffer(&mut self, queue: &wgpu::Queue, id: u32, buffer: &[u8]) -> Result<u32, &'static str> {
		if let Some(held_buffer) = self.held_buffers.get_mut(&id) {
			if held_buffer.size >= buffer.len() as u32 {
				self.held_bytes -= held_buffer.size;
				self.held_bytes += buffer.len() as u32;
				self.held_bytes_alignment -= held_buffer.size.next_multiple_of(self.alignment as u32);
				self.held_bytes_alignment += (buffer.len() as u32).next_multiple_of(self.alignment as u32);
				held_buffer.size = buffer.len() as u32;
				queue.write_buffer(&self.buffer, held_buffer.offset as u64, buffer);
				Ok(id)
			} else {
				self.remove_buffer(id)?;
				self.add_buffer(queue, buffer)
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

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Copy, PureClone)]
pub struct PackedBufferGroupId {
	internal_id: u32,
	buffer_index: u16,
}

impl PackedBufferGroupId {
	pub fn new(internal_id: u32, buffer_index: u16) -> Self {
		Self {
			internal_id,
			buffer_index
		}
	}

	pub fn internal_id(&self) -> u32 { self.internal_id }
	pub fn buffer_index(&self) -> u16 { self.buffer_index }
}

pub struct PackedBufferGroup {
	buffers: Vec<PackedBuffer>,
	buffer_size: u32,
	alignment: u32,
	usage: wgpu::BufferUsages,
}

impl PackedBufferGroup {
	pub fn new(buffer_size: u32, alignment: u32, usage: wgpu::BufferUsages) -> Result<Self, &'static str> {
		Ok(Self {
			buffers: vec![],
			buffer_size,
			alignment,
			usage,
		})
	}

	pub fn add_buffer(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, buffer: &[u8]) -> Result<PackedBufferGroupId, &'static str> {
		if buffer.len() as u32 > self.buffer_size {
			return Err("Buffer is larger than buffer max size.");
		}
		for (i, packed_buffer) in self.buffers.iter_mut().enumerate() {
			if let Ok(id) = packed_buffer.add_buffer(queue, buffer) {
				return Ok(PackedBufferGroupId::new(id, i as u16));
			}
		}
		self.buffers.push(PackedBuffer::new(device, self.buffer_size, self.alignment, self.usage)?);
		return Ok(PackedBufferGroupId::new(self.buffers.last_mut().unwrap().add_buffer(queue, buffer)?, self.buffers.len() as u16 - 1));
	}

	pub fn remove_buffer(&mut self, id: PackedBufferGroupId) -> Result<(), &'static str> {
		if let Some(packed_buffer) = self.buffers.get_mut(id.buffer_index as usize) {
			return packed_buffer.remove_buffer(id.internal_id);
		}
		return Err("Could not find id.");
	}

	pub fn replace_buffer(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, id: PackedBufferGroupId, buffer: &[u8]) -> Result<PackedBufferGroupId, &'static str> {
		if buffer.len() as u32 > self.buffer_size {
			return Err("Buffer is larger than buffer max size.");
		}
		if let Some(packed_buffer) = self.buffers.get_mut(id.buffer_index as usize) {
			if let Ok(new_internal_id) = packed_buffer.replace_buffer(queue, id.internal_id, buffer) {
				return Ok(PackedBufferGroupId::new(new_internal_id, id.buffer_index));
			}
			return self.add_buffer(device, queue, buffer);
		}
		return Err("Could not find id.");
	}

	pub fn get_packed_buffer(&self, buffer_index: u16) -> Option<&PackedBuffer> {
		self.buffers.get(buffer_index as usize)
	}

	pub fn get_held_buffer(&self, id: PackedBufferGroupId) -> Option<&HeldBuffer> {
		self.buffers.get(id.buffer_index as usize)?.get_held_buffer(id.internal_id)
	}

	pub fn get_buffer(&self, buffer_index: u16) -> Option<&wgpu::Buffer> {
		Some(self.buffers.get(buffer_index as usize)?.get_buffer())
	}
}
