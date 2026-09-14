use bevy::render::renderer::WgpuWrapper;
use wgpu::util::DeviceExt;

use crate::shader_sources::VoxelShaderSources;

pub const GI_ENTRIES: [&str; 7] = [
	"discover_visible", "prepare_visible", "trace_indirect", "discover_contributors",
	"prepare_all", "light_direct", "gather_indirect",
];
const HEADER_WORDS: u32 = 16;
const MIN_VISIBLE: u32 = 64;
const MAX_VISIBLE: u32 = 262_144;
const RAYS_PER_FACE: u32 = 8;

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GiParams {
	pub sizes: [u32; 4],
	pub offsets: [u32; 4],
}

#[derive(Clone, Copy, Debug)]
pub struct GiArenaLayout {
	pub params: GiParams,
	pub byte_size: u64,
	pub clear_bytes: u64,
}

impl GiArenaLayout {
	pub fn new(width: u32, height: u32, limits: &wgpu::Limits) -> anyhow::Result<Self> {
		anyhow::ensure!(width > 0 && height > 0, "GI intermediate dimensions must be nonzero");
		anyhow::ensure!(limits.max_bind_groups >= 4 && limits.max_storage_buffers_per_shader_stage >= 8,
			"face GI requires four bind groups and eight storage buffers per stage");
		anyhow::ensure!(limits.max_compute_invocations_per_workgroup >= 64
			&& limits.max_compute_workgroup_size_x >= 64 && limits.max_compute_workgroup_size_y >= 4,
			"face GI requires 64-thread compute workgroups");
		anyhow::ensure!(width.div_ceil(8) <= limits.max_compute_workgroups_per_dimension
			&& height.div_ceil(4) <= limits.max_compute_workgroups_per_dimension,
			"GI discovery dispatch exceeds device limits");
		let pixels = u64::from(width) * u64::from(height);
		let primary_references = pixels.checked_mul(3)
			.ok_or_else(|| anyhow::anyhow!("GI primary reference count overflow"))?;
		let mut visible = pixels.min(u64::from(MAX_VISIBLE)).next_power_of_two().max(u64::from(MIN_VISIBLE)) as u32;
		let byte_limit = limits.max_buffer_size.min(limits.max_storage_buffer_binding_size);
		loop {
			let layout = Self::with_capacity(width, height, visible)?;
			let reference_end = primary_references.checked_add(u64::from(visible) * u64::from(RAYS_PER_FACE));
			let references_fit = reference_end.is_some_and(|end| end < u64::from(u32::MAX) - 1);
			let dispatch_fits = (visible * RAYS_PER_FACE).div_ceil(64) <= limits.max_compute_workgroups_per_dimension;
			if layout.byte_size <= byte_limit && references_fit && dispatch_fits {
				return Ok(layout);
			}
			anyhow::ensure!(visible > MIN_VISIBLE,
				"device limits or dimensions cannot fit the minimum face GI arena");
			visible /= 2;
		}
	}

	fn with_capacity(width: u32, height: u32, visible: u32) -> anyhow::Result<Self> {
		let table = visible.checked_mul(8).ok_or_else(|| anyhow::anyhow!("GI table size overflow"))?;
		let faces = HEADER_WORDS.checked_add(table);
		let hits = faces.and_then(|offset| offset.checked_add(table));
		let lighting = hits.and_then(|offset| visible.checked_mul(32).and_then(|words| offset.checked_add(words)));
		let end = lighting.and_then(|offset| table.checked_mul(7).and_then(|words| offset.checked_add(words)));
		let end = end.ok_or_else(|| anyhow::anyhow!("GI arena word offsets overflow"))?;
		Ok(Self {
			params: GiParams {
				sizes: [table, visible, width, height],
				offsets: [HEADER_WORDS, faces.unwrap(), hits.unwrap(), lighting.unwrap()],
			},
			byte_size: u64::from(end) * 4,
			clear_bytes: u64::from(faces.unwrap()) * 4,
		})
	}
}

#[derive(Default, Clone, Copy, Debug)]
pub struct FaceGiStats {
	pub enabled: bool,
	pub arena_bytes: u64,
	pub visible_capacity: u32,
	pub table_capacity: u32,
	pub visible_faces: u32,
	pub total_faces: u32,
	pub secondary_hits: u32,
	pub incomplete_rays: u32,
	pub sky_misses: u32,
	pub overflow_flags: u32,
}

pub struct FaceGiReadback {
	pub buffer: WgpuWrapper<wgpu::Buffer>,
	pub stats: FaceGiStats,
}

impl FaceGiReadback {
	pub fn read_mapped(&self) -> FaceGiStats {
		let view = self.buffer.slice(..).get_mapped_range();
		let words: &[u32] = bytemuck::cast_slice(&view);
		FaceGiStats {
			total_faces: words[0], visible_faces: words[1], overflow_flags: words[2],
			incomplete_rays: words[3], secondary_hits: words[13], sky_misses: words[14],
			..self.stats
		}
	}
}

pub(crate) fn storage_entry(binding: u32, visibility: wgpu::ShaderStages, read_only: bool) -> wgpu::BindGroupLayoutEntry {
	wgpu::BindGroupLayoutEntry {
		binding, visibility,
		ty: wgpu::BindingType::Buffer {
			ty: wgpu::BufferBindingType::Storage { read_only },
			has_dynamic_offset: false, min_binding_size: None,
		},
		count: None,
	}
}

pub(crate) fn params_entry(visibility: wgpu::ShaderStages) -> wgpu::BindGroupLayoutEntry {
	wgpu::BindGroupLayoutEntry {
		binding: 2, visibility,
		ty: wgpu::BindingType::Buffer {
			ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false,
			min_binding_size: wgpu::BufferSize::new(size_of::<GiParams>() as u64),
		},
		count: None,
	}
}

pub struct FaceGi {
	pub words: WgpuWrapper<wgpu::Buffer>,
	pub params: WgpuWrapper<wgpu::Buffer>,
	pub arena: GiArenaLayout,
	indirect: WgpuWrapper<wgpu::Buffer>,
	geometry_layout: WgpuWrapper<wgpu::BindGroupLayout>,
	bind_group: WgpuWrapper<wgpu::BindGroup>,
	pipelines: [WgpuWrapper<wgpu::ComputePipeline>; 7],
}

impl FaceGi {
	pub fn new(
		device: &wgpu::Device,
		intermediate: &wgpu::Texture,
		camera_layout: &wgpu::BindGroupLayout,
		bvh_layout: &wgpu::BindGroupLayout,
		shaders: &VoxelShaderSources,
	) -> anyhow::Result<Self> {
		let arena = GiArenaLayout::new(intermediate.width(), intermediate.height(), &device.limits())?;
		let words = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("face_gi_words"), size: arena.byte_size,
			usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		}));
		let params = WgpuWrapper::new(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("face_gi_params"), contents: bytemuck::bytes_of(&arena.params),
			usage: wgpu::BufferUsages::UNIFORM,
		}));
		// Indirect arguments must not alias writable shader storage.
		let indirect = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("face_gi_indirect"), size: 36,
			usage: wgpu::BufferUsages::INDIRECT | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		}));
		let geometry_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: Some("face_gi_geometry_layout"),
			entries: &std::array::from_fn::<_, 4, _>(|i| storage_entry(i as u32, wgpu::ShaderStages::COMPUTE, true)),
		}));
		let scratch_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: Some("face_gi_scratch_layout"),
			entries: &[
				wgpu::BindGroupLayoutEntry {
					binding: 0, visibility: wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Texture {
						sample_type: wgpu::TextureSampleType::Uint,
						view_dimension: wgpu::TextureViewDimension::D2, multisampled: false,
					},
					count: None,
				},
				storage_entry(1, wgpu::ShaderStages::COMPUTE, false),
				params_entry(wgpu::ShaderStages::COMPUTE),
			],
		});
		let view = intermediate.create_view(&wgpu::TextureViewDescriptor {
			usage: Some(wgpu::TextureUsages::TEXTURE_BINDING), ..Default::default()
		});
		let bind_group = WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			label: Some("face_gi_scratch"), layout: &scratch_layout,
			entries: &[
				wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&view) },
				wgpu::BindGroupEntry { binding: 1, resource: words.as_entire_binding() },
				wgpu::BindGroupEntry { binding: 2, resource: params.as_entire_binding() },
			],
		}));
		let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: Some("face_gi_pipeline_layout"),
			bind_group_layouts: &[Some(camera_layout), Some(bvh_layout), Some(&geometry_layout), Some(&scratch_layout)],
			immediate_size: 0,
		});
		let pipelines = std::array::from_fn(|i| {
			let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
				label: Some(GI_ENTRIES[i]), source: wgpu::ShaderSource::Wgsl(shaders.face_gi[i].clone().into()),
			});
			WgpuWrapper::new(device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
				label: Some(GI_ENTRIES[i]), layout: Some(&pipeline_layout), module: &shader,
				entry_point: Some(GI_ENTRIES[i]), compilation_options: Default::default(), cache: None,
			}))
		});
		Ok(Self { words, params, arena, indirect, geometry_layout, bind_group, pipelines })
	}

	pub fn clear(&self, encoder: &mut wgpu::CommandEncoder, enabled: bool) {
		encoder.clear_buffer(&self.words, 0, Some(if enabled { self.arena.clear_bytes } else { u64::from(HEADER_WORDS) * 4 }));
	}

	pub fn dispatch(
		&self,
		device: &wgpu::Device,
		encoder: &mut wgpu::CommandEncoder,
		camera: &wgpu::BindGroup,
		view_offset: u32,
		bvh: &wgpu::BindGroup,
		geometry: [&wgpu::Buffer; 4],
	) {
		let geometry = device.create_bind_group(&wgpu::BindGroupDescriptor {
			label: Some("face_gi_geometry"), layout: &self.geometry_layout,
			entries: &std::array::from_fn::<_, 4, _>(|i| wgpu::BindGroupEntry {
				binding: i as u32, resource: geometry[i].as_entire_binding(),
			}),
		});
		for i in 0..GI_ENTRIES.len() {
			{
				let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
					label: Some(GI_ENTRIES[i]), timestamp_writes: None,
				});
				pass.set_bind_group(0, camera, &[view_offset]);
				pass.set_bind_group(1, bvh, &[]);
				pass.set_bind_group(2, &geometry, &[]);
				pass.set_bind_group(3, &*self.bind_group, &[]);
				pass.set_pipeline(&self.pipelines[i]);
				match i {
					0 => pass.dispatch_workgroups(self.arena.params.sizes[2].div_ceil(8), self.arena.params.sizes[3].div_ceil(4), 1),
					1 | 4 => pass.dispatch_workgroups(1, 1, 1),
					2 | 3 => pass.dispatch_workgroups_indirect(&self.indirect, 12),
					5 => pass.dispatch_workgroups_indirect(&self.indirect, 24),
					6 => pass.dispatch_workgroups_indirect(&self.indirect, 0),
					_ => unreachable!(),
				}
			}
			match i {
				1 => encoder.copy_buffer_to_buffer(&self.words, 4 * 4, &self.indirect, 0, 36),
				4 => encoder.copy_buffer_to_buffer(&self.words, 10 * 4, &self.indirect, 24, 12),
				_ => {},
			}
		}
	}

	pub fn copy_stats(&self, device: &wgpu::Device, encoder: &mut wgpu::CommandEncoder, enabled: bool) -> FaceGiReadback {
		let buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("face_gi_stats"), size: u64::from(HEADER_WORDS) * 4,
			usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST, mapped_at_creation: false,
		}));
		encoder.copy_buffer_to_buffer(&self.words, 0, &buffer, 0, buffer.size());
		FaceGiReadback {
			buffer,
			stats: FaceGiStats {
				enabled, arena_bytes: self.arena.byte_size,
				visible_capacity: self.arena.params.sizes[1], table_capacity: self.arena.params.sizes[0],
				..Default::default()
			},
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn arena_ranges_are_contiguous_and_bounded() {
		for (width, height) in [(1, 1), (9, 9), (1921, 1081), (3841, 2161)] {
			let layout = GiArenaLayout::new(width, height, &wgpu::Limits::default()).unwrap();
			let [table, visible, w, h] = layout.params.sizes;
			let [table_start, faces, hits, lighting] = layout.params.offsets;
			assert_eq!((w, h), (width, height));
			assert!(visible.is_power_of_two() && (MIN_VISIBLE..=MAX_VISIBLE).contains(&visible));
			assert_eq!(table, visible * 8);
			assert_eq!(table_start, HEADER_WORDS);
			assert_eq!(faces, table_start + table);
			assert_eq!(hits, faces + table);
			assert_eq!(lighting, hits + visible * RAYS_PER_FACE * 4);
			assert_eq!(layout.byte_size, u64::from(lighting + table * 7) * 4);
			assert_eq!(layout.clear_bytes, u64::from(faces) * 4);
			assert!(layout.clear_bytes < layout.byte_size);
			let reference_end = u64::from(w) * u64::from(h) * 3 + u64::from(visible) * 8;
			assert!(reference_end < u64::from(u32::MAX) - 1);
			let words: &[u32] = bytemuck::cast_slice(std::slice::from_ref(&layout.params));
			assert_eq!(words, &[table, visible, w, h, table_start, faces, hits, lighting]);
		}
		assert_eq!(GiArenaLayout::new(1, 1, &wgpu::Limits::default()).unwrap().params.sizes[1], MIN_VISIBLE);
		assert_eq!(GiArenaLayout::new(9, 9, &wgpu::Limits::default()).unwrap().params.sizes[1], 128);
		let layout = GiArenaLayout::new(1921, 1081, &wgpu::Limits::default()).unwrap();
		assert_eq!(layout.params.sizes[1], MAX_VISIBLE);
		assert_eq!(layout.byte_size, 104 * 1024 * 1024 + 64);
		assert_eq!(size_of::<GiParams>(), 32);
	}

	#[test]
	fn low_buffer_limits_reduce_capacity() {
		let target = GiArenaLayout::with_capacity(1024, 1024, 128).unwrap();
		for storage_limit in [true, false] {
			let mut limits = wgpu::Limits::default();
			if storage_limit { limits.max_storage_buffer_binding_size = target.byte_size; }
			else { limits.max_buffer_size = target.byte_size; }
			let layout = GiArenaLayout::new(1024, 1024, &limits).unwrap();
			assert_eq!(layout.params.sizes[1], 128);
			assert_eq!(layout.byte_size, target.byte_size);
		}
	}

	#[test]
	fn impossible_limits_and_reference_overflow_are_rejected() {
		let mut limits = wgpu::Limits::default();
		limits.max_storage_buffer_binding_size = GiArenaLayout::with_capacity(1, 1, MIN_VISIBLE).unwrap().byte_size - 1;
		assert!(GiArenaLayout::new(1, 1, &limits).is_err());
		let mut limits = wgpu::Limits::default();
		limits.max_storage_buffers_per_shader_stage = 7;
		assert!(GiArenaLayout::new(1, 1, &limits).is_err());
		let mut limits = wgpu::Limits::default();
		limits.max_bind_groups = 3;
		assert!(GiArenaLayout::new(1, 1, &limits).is_err());
		let limits = wgpu::Limits::default();
		assert!(GiArenaLayout::new(0, 1, &limits).is_err());
		assert!(GiArenaLayout::new(65536, 65536, &limits).is_err());
		assert!(GiArenaLayout::with_capacity(1, 1, u32::MAX).is_err());
	}

	#[test]
	fn low_dispatch_limits_reduce_capacity() {
		let mut limits = wgpu::Limits::default();
		limits.max_compute_workgroups_per_dimension = 256;
		let layout = GiArenaLayout::new(1024, 1024, &limits).unwrap();
		assert_eq!(layout.params.sizes[1], 2048);
		assert!(GiArenaLayout::new(1024, 1025, &limits).is_err());
	}
}
