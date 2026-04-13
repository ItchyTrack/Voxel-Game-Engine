use std::collections::BTreeMap;

use etagere::{size2, AllocId, AtlasAllocator};

const TEXTURE_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::R32Uint;
const BYTES_PER_TEXEL: u32 = 4; // R32Uint

pub struct HeldTextureAllocation {
    atlas_allocation_id: AllocId,
    texture_rectangle: etagere::Rectangle,
    logical_texel_width: u32,
    logical_texel_height: u32,
    original_byte_length: u32,
}

impl HeldTextureAllocation {
    /// The rectangle within the atlas texture where this allocation lives.
    /// `min` is the top-left corner in texel coordinates; `max` is exclusive.
    pub fn texture_rectangle(&self) -> etagere::Rectangle {
        self.texture_rectangle
    }

    /// Width of the 2D region in texels. Each texel holds 4 bytes (R32Uint).
    pub fn logical_texel_width(&self) -> u32 {
        self.logical_texel_width
    }

    /// Height of the 2D region in texels.
    pub fn logical_texel_height(&self) -> u32 {
        self.logical_texel_height
    }

    /// The original 1D byte length passed to `add_buffer` or `replace_buffer`.
    /// The last texel of the 2D region may be partially filled; bytes beyond
    /// this length within that texel are padding zeros.
    pub fn original_byte_length(&self) -> u32 {
        self.original_byte_length
    }
}

pub struct PackedBufferTexture {
    texture: wgpu::Texture,
    texture_view: wgpu::TextureView,
    atlas_allocator: AtlasAllocator,
    texture_side_length: u32,
    held_allocations: BTreeMap<u32, HeldTextureAllocation>,
    next_allocation_id: u32,
}

impl PackedBufferTexture {
    pub fn new(device: &wgpu::Device, texture_side_length: u32, usage: wgpu::TextureUsages) -> Self {
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("PackedBufferTexture"),
            size: wgpu::Extent3d {
                width: texture_side_length,
                height: texture_side_length,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: TEXTURE_FORMAT,
            usage: usage | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let texture_view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let atlas_allocator =
            AtlasAllocator::new(size2(texture_side_length as i32, texture_side_length as i32));
        Self {
            texture,
            texture_view,
            atlas_allocator,
            texture_side_length,
            held_allocations: BTreeMap::new(),
            next_allocation_id: 0,
        }
    }

    pub fn texture_side_length(&self) -> u32 {
        self.texture_side_length
    }

    pub fn get_texture(&self) -> &wgpu::Texture {
        &self.texture
    }

    pub fn get_texture_view(&self) -> &wgpu::TextureView {
        &self.texture_view
    }

    pub fn get_held_allocation(&self, id: u32) -> Option<&HeldTextureAllocation> {
        self.held_allocations.get(&id)
    }

    pub fn add_buffer(
        &mut self,
        queue: &wgpu::Queue,
        source_data_buffer: &[u8],
    ) -> Result<u32, &'static str> {
        if source_data_buffer.is_empty() {
            return Err("Buffer size can't be 0.");
        }

        let (logical_texel_width, logical_texel_height) =
            Self::compute_squarish_dimensions_for_byte_count(source_data_buffer.len() as u32);

        let atlas_allocation = self
            .atlas_allocator
            .allocate(size2(logical_texel_width as i32, logical_texel_height as i32))
            .ok_or("Texture atlas is full.")?;

        let allocation_texture_rectangle = atlas_allocation.rectangle;
        Self::write_1d_buffer_into_2d_texture_region(
            queue,
            &self.texture,
            source_data_buffer,
            allocation_texture_rectangle.min.x as u32,
            allocation_texture_rectangle.min.y as u32,
            logical_texel_width,
            logical_texel_height,
        );

        let allocation_handle_id = self.next_allocation_id;
        self.next_allocation_id += 1;
        self.held_allocations.insert(
            allocation_handle_id,
            HeldTextureAllocation {
                atlas_allocation_id: atlas_allocation.id,
                texture_rectangle: allocation_texture_rectangle,
                logical_texel_width,
                logical_texel_height,
                original_byte_length: source_data_buffer.len() as u32,
            },
        );

        Ok(allocation_handle_id)
    }

    pub fn remove_buffer(&mut self, id: u32) -> Result<(), &'static str> {
        if let Some(held_allocation) = self.held_allocations.remove(&id) {
            self.atlas_allocator
                .deallocate(held_allocation.atlas_allocation_id);
            Ok(())
        } else {
            Err("Could not find id.")
        }
    }

    // If the new buffer does not fit the old slot, the old allocation will
    // still be removed and a new one will be inserted at a different location.
    pub fn replace_buffer(
        &mut self,
        queue: &wgpu::Queue,
        id: u32,
        new_data_buffer: &[u8],
    ) -> Result<u32, &'static str> {
        let (new_logical_texel_width, new_logical_texel_height) =
            Self::compute_squarish_dimensions_for_byte_count(new_data_buffer.len() as u32);

        let (existing_logical_texel_width, existing_logical_texel_height, existing_rectangle) =
            match self.held_allocations.get(&id) {
                Some(held_allocation) => (
                    held_allocation.logical_texel_width,
                    held_allocation.logical_texel_height,
                    held_allocation.texture_rectangle,
                ),
                None => return Err("Could not find id."),
            };

        let fits_existing_slot = new_logical_texel_width == existing_logical_texel_width
            && new_logical_texel_height == existing_logical_texel_height;

        if fits_existing_slot {
            Self::write_1d_buffer_into_2d_texture_region(
                queue,
                &self.texture,
                new_data_buffer,
                existing_rectangle.min.x as u32,
                existing_rectangle.min.y as u32,
                new_logical_texel_width,
                new_logical_texel_height,
            );
            let held_allocation = self.held_allocations.get_mut(&id).unwrap();
            held_allocation.original_byte_length = new_data_buffer.len() as u32;
            Ok(id)
        } else {
            self.remove_buffer(id)?;
            self.add_buffer(queue, new_data_buffer)
        }
    }

    /// Wraps a flat byte count into a squarish texel grid. The byte count is
    /// first converted to a texel count (rounding up to the nearest u32), then
    /// the texel grid is made as square as possible.
    /// For example, 16 bytes → 4 texels → (2, 2); 20 bytes → 5 texels → (3, 2).
    fn compute_squarish_dimensions_for_byte_count(byte_count: u32) -> (u32, u32) {
        let texel_count = byte_count.div_ceil(BYTES_PER_TEXEL);
        let logical_texel_width = (texel_count as f64).sqrt().ceil() as u32;
        let logical_texel_height = texel_count.div_ceil(logical_texel_width);
        (logical_texel_width, logical_texel_height)
    }

    /// Reinterprets a 1D byte slice as a 2D grid of R32Uint texels and uploads
    /// it into the given sub-rectangle of the atlas texture. Each texel holds 4
    /// bytes; the final texel of the last row may be partially filled and is
    /// zero-padded. Rows are also padded to satisfy wgpu's
    /// `COPY_BYTES_PER_ROW_ALIGNMENT` requirement.
    fn write_1d_buffer_into_2d_texture_region(
        queue: &wgpu::Queue,
        destination_texture: &wgpu::Texture,
        source_data_buffer: &[u8],
        destination_origin_x: u32,
        destination_origin_y: u32,
        logical_texel_width: u32,
        logical_texel_height: u32,
    ) {
        let source_bytes_per_row = logical_texel_width * BYTES_PER_TEXEL;
        let bytes_per_row_aligned_to_copy_alignment =
            source_bytes_per_row.next_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);

        let padded_staging_buffer_size =
            (bytes_per_row_aligned_to_copy_alignment * logical_texel_height) as usize;
        let mut padded_staging_buffer = vec![0u8; padded_staging_buffer_size];

        for row_index in 0..logical_texel_height {
            let source_row_byte_start = (row_index * source_bytes_per_row) as usize;
            let source_row_byte_end = ((row_index + 1) * source_bytes_per_row) as usize;
            let source_row_byte_end_clamped = source_row_byte_end.min(source_data_buffer.len());

            if source_row_byte_start >= source_data_buffer.len() {
                break;
            }

            let staging_row_byte_start =
                (row_index * bytes_per_row_aligned_to_copy_alignment) as usize;
            let row_byte_copy_count = source_row_byte_end_clamped - source_row_byte_start;

            padded_staging_buffer
                [staging_row_byte_start..staging_row_byte_start + row_byte_copy_count]
                .copy_from_slice(
                    &source_data_buffer[source_row_byte_start..source_row_byte_end_clamped],
                );
        }

        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: destination_texture,
                mip_level: 0,
                origin: wgpu::Origin3d {
                    x: destination_origin_x,
                    y: destination_origin_y,
                    z: 0,
                },
                aspect: wgpu::TextureAspect::All,
            },
            &padded_staging_buffer,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(bytes_per_row_aligned_to_copy_alignment),
                rows_per_image: Some(logical_texel_height),
            },
            wgpu::Extent3d {
                width: logical_texel_width,
                height: logical_texel_height,
                depth_or_array_layers: 1,
            },
        );
    }
}
