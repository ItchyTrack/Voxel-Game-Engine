use std::collections::BTreeMap;

pub struct HeldBuffer {
	offset: u64,
	size: u64,
}

pub struct PackedBuffer {
	buffer: wgpu::Buffer,
	held_bytes: u64,
	alignment: u16,
	held_buffers: BTreeMap<u8, HeldBuffer>,
}

impl PackedBuffer {
	pub fn new(device: &wgpu::Device, label: wgpu::Label, size: u64, alignment: u16, usage: wgpu::BufferUsages) -> Result<Self, &'static str> {
		if !size.is_multiple_of(alignment as u64) {
			return Err("Size must be a multiple of alignment.");
		}
		let buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label,
			size,
			usage,
			mapped_at_creation: false,
		});
		Ok(Self {
			buffer,
			held_bytes: 0,
			alignment,
			held_buffers: BTreeMap::new()
		})
	}

	pub fn add_buffer(&mut self, queue: &wgpu::Queue, buffer: &[u8]) -> Result<u8, &'static str> {
		if buffer.len() == 0 {
			return Err("Buffer size can't be 0.");
		}
		if buffer.len() as u64 > self.buffer.size() - self.held_bytes {
			return Err("Not enought free space.");
		}
		let mut placement_location = 0;
		let mut after_buffer_iter = self.held_buffers.iter();
		loop {
			let mut failed = false;
			let buffer_end = placement_location + buffer.len() as u64;
			for (_, held_buffer) in &self.held_buffers {
				// held:	|-----...											|------...										...------|
				// old:		|-----...											...------|										|------...
				if held_buffer.offset == placement_location || (held_buffer.offset < buffer_end && placement_location < held_buffer.offset + held_buffer.size) {
					failed = true;
					break;
				}
			}
			if failed {
				if placement_location == 0 {
					assert!(self.held_buffers.len() != 0); // because how would it fail here?
				}
				loop {
					if let Some((_, held_buffer)) = after_buffer_iter.next() {
						let after_buffer_end = held_buffer.offset + held_buffer.size;
						placement_location = after_buffer_end.div_ceil(self.alignment as u64) * self.alignment as u64;
						if (placement_location + buffer.len() as u64) < (self.buffer.size() as u64) {
							break;
						}
					} else {
						return Err("Not enought free space.");
					}
				}
				continue;
			}
			break;
		}
		assert!(placement_location.is_multiple_of(self.alignment as u64));
		queue.write_buffer(&self.buffer, placement_location, buffer);
		self.held_bytes += (buffer.len() as u64).next_multiple_of(self.alignment as u64);
		let mut id = 0;
		for (held_id, _) in self.held_buffers.iter() {
			if *held_id == id {
				if id == u8::MAX {
					return Err("Max buffer count reached.");
				}
				id += 1;
			} else {
				break;
			}
		}
		self.held_buffers.insert(id, HeldBuffer { offset: placement_location, size: buffer.len() as u64 });
		Ok(id)
	}

	pub fn remove_buffer(&mut self, id: u8) -> Result<(), &'static str> {
		if let Some(held_buffer) = self.held_buffers.remove(&id) {
			self.held_bytes -= held_buffer.size.next_multiple_of(self.alignment as u64);
			Ok(())
		} else {
			Err("Could not find id.")
		}
	}

	pub fn replace_buffer(&mut self, queue: &wgpu::Queue, id: u8, buffer: &[u8]) -> Result<(), &'static str> {
		if let Some(held_buffer) = self.held_buffers.get_mut(&id) {
			if held_buffer.size >= buffer.len() as u64 {
				self.held_bytes -= held_buffer.size.next_multiple_of(self.alignment as u64);
				self.held_bytes += (buffer.len() as u64).next_multiple_of(self.alignment as u64);
				held_buffer.size = buffer.len() as u64;
				queue.write_buffer(&self.buffer, held_buffer.offset, buffer);
				Ok(())
			} else {
				self.remove_buffer(id)?;
				self.add_buffer_with_id(queue, buffer, id)
			}
		} else {
			Err("Could not find id.")
		}
	}

	// used when replacing buffers
	fn add_buffer_with_id(&mut self, queue: &wgpu::Queue, buffer: &[u8], id: u8) -> Result<(), &'static str> {
		if buffer.len() == 0 {
			return Err("Buffer size can't be 0.");
		}
		if buffer.len() as u64 > self.buffer.size() - self.held_bytes {
			return Err("Not enought free space.");
		}
		let mut placement_location = 0;
		let mut after_buffer_iter = self.held_buffers.iter();
		loop {
			let mut failed = false;
			let buffer_end = placement_location + buffer.len() as u64;
			for (_, held_buffer) in &self.held_buffers {
				// held:	|-----...											|------...										...------|
				// old:		|-----...											...------|										|------...
				if held_buffer.offset == placement_location || (held_buffer.offset < buffer_end && placement_location < held_buffer.offset + held_buffer.size) {
					failed = true;
					break;
				}
			}
			if failed {
				if placement_location == 0 {
					assert!(self.held_buffers.len() != 0); // because how would it fail here?
				}
				loop {
					if let Some((_, held_buffer)) = after_buffer_iter.next() {
						let after_buffer_end = held_buffer.offset + held_buffer.size;
						placement_location = after_buffer_end.div_ceil(self.alignment as u64) * self.alignment as u64;
						if (placement_location + buffer.len() as u64) < (self.buffer.size() as u64) {
							break;
						}
					} else {
						return Err("Not enought free space.");
					}
				}
				continue;
			}
			break;
		}
		assert!(placement_location.is_multiple_of(self.alignment as u64));
		queue.write_buffer(&self.buffer, placement_location, buffer);
		self.held_bytes += (buffer.len() as u64).next_multiple_of(self.alignment as u64);
		self.held_buffers.insert(id, HeldBuffer { offset: placement_location, size: buffer.len() as u64 });
		Ok(())
	}
}

// struct PackedBufferGroup {
// 	buffers: Vec<PackedBuffer>,
// 	buffer_size: u64,
// 	alignment: u16,
// 	usage: wgpu::BufferUsages,
// }

// impl PackedBufferGroup {
// 	pub fn new(buffer_size: u64, alignment: u16, usage: wgpu::BufferUsages) -> Result<Self, &'static str> {
// 		if !buffer_size.is_multiple_of(alignment as u64) {
// 			return Err("Size must be a multiple of alignment.");
// 		}
// 		Ok(Self {
// 			buffers: vec![],
// 			buffer_size,
// 			alignment,
// 			usage,
// 		})
// 	}

// 	pub fn add_buffer(&mut self) -> Result<u8, &'static str> {

// 	}
// }
