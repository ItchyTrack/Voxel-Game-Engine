use bevy::render::renderer::WgpuWrapper;
use wgpu::util::DeviceExt;

use crate::graphics_settings::{GiRayCount, LightingMode};
use crate::shader_sources::VoxelShaderSources;

pub const GI_ENTRIES: [&str; 7] = [
	"discover_visible", "prepare_visible", "trace_indirect", "discover_contributors",
	"prepare_all", "light_direct", "gather_indirect",
];
const HEADER_WORDS: u32 = 16;
const MIN_VISIBLE: u32 = 64;
const MAX_VISIBLE: u32 = 262_144;

fn pass_indices(lighting: LightingMode) -> &'static [usize] {
	match lighting {
		LightingMode::None => &[],
		LightingMode::Direct => &[0, 1, 4, 5],
		LightingMode::Indirect => &[0, 1, 2, 3, 4, 5, 6],
	}
}

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GiParams {
	pub sizes: [u32; 4],
	pub offsets: [u32; 4],
	pub settings: [u32; 4],
}

#[derive(Clone, Copy, Debug)]
pub struct GiArenaLayout {
	pub params: GiParams,
	pub byte_size: u64,
	pub clear_bytes: u64,
}

impl GiArenaLayout {
	pub fn new(width: u32, height: u32, ray_count: GiRayCount, limits: &wgpu::Limits) -> anyhow::Result<Self> {
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
			let layout = Self::with_capacity(width, height, visible, ray_count, limits.max_compute_workgroups_per_dimension)?;
			let rays = u64::from(visible) * u64::from(ray_count.count());
			let reference_end = primary_references.checked_add(rays);
			let references_fit = reference_end.is_some_and(|end| end < u64::from(u32::MAX) - 1);
			let max_groups = u64::from(limits.max_compute_workgroups_per_dimension).pow(2);
			let dispatch_fits = rays.div_ceil(64) <= max_groups
				&& u64::from(layout.params.sizes[0]).div_ceil(64) <= max_groups;
			if layout.byte_size <= byte_limit && references_fit && dispatch_fits {
				return Ok(layout);
			}
			anyhow::ensure!(visible > MIN_VISIBLE,
				"device limits or dimensions cannot fit the minimum face GI arena");
			visible /= 2;
		}
	}

	fn with_capacity(width: u32, height: u32, visible: u32, ray_count: GiRayCount, max_dispatch_dimension: u32) -> anyhow::Result<Self> {
		let table = visible.checked_mul(8).ok_or_else(|| anyhow::anyhow!("GI table size overflow"))?;
		let faces = HEADER_WORDS.checked_add(table);
		let hits = faces.and_then(|offset| offset.checked_add(table));
		let hit_words = visible.checked_mul(ray_count.count()).and_then(|rays| rays.checked_mul(4));
		let lighting = hits.and_then(|offset| hit_words.and_then(|words| offset.checked_add(words)));
		let end = lighting.and_then(|offset| table.checked_mul(7).and_then(|words| offset.checked_add(words)));
		let end = end.ok_or_else(|| anyhow::anyhow!("GI arena word offsets overflow"))?;
		Ok(Self {
			params: GiParams {
				sizes: [table, visible, width, height],
				offsets: [HEADER_WORDS, faces.unwrap(), hits.unwrap(), lighting.unwrap()],
				settings: [ray_count.count(), max_dispatch_dimension, 0, 0],
			},
			byte_size: u64::from(end) * 4,
			clear_bytes: u64::from(faces.unwrap()) * 4,
		})
	}
}

#[derive(Default, Clone, Copy, Debug)]
pub struct FaceGiStats {
	pub enabled: bool,
	pub indirect: bool,
	pub rays_per_face: u32,
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
		ray_count: GiRayCount,
	) -> anyhow::Result<Self> {
		let arena = GiArenaLayout::new(intermediate.width(), intermediate.height(), ray_count, &device.limits())?;
		let words = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("face_gi_words"), size: arena.byte_size,
			usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		}));
		let params = WgpuWrapper::new(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("face_gi_params"), contents: bytemuck::bytes_of(&arena.params),
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
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

	pub fn clear(&self, encoder: &mut wgpu::CommandEncoder, lighting: LightingMode) {
		encoder.clear_buffer(&self.words, 0, Some(if lighting != LightingMode::None { self.arena.clear_bytes } else { u64::from(HEADER_WORDS) * 4 }));
	}

	pub fn dispatch(
		&self,
		device: &wgpu::Device,
		encoder: &mut wgpu::CommandEncoder,
		camera: &wgpu::BindGroup,
		view_offset: u32,
		bvh: &wgpu::BindGroup,
		geometry: [&wgpu::Buffer; 4],
		lighting: LightingMode,
	) {
		if lighting == LightingMode::None { return; }
		let geometry = device.create_bind_group(&wgpu::BindGroupDescriptor {
			label: Some("face_gi_geometry"), layout: &self.geometry_layout,
			entries: &std::array::from_fn::<_, 4, _>(|i| wgpu::BindGroupEntry {
				binding: i as u32, resource: geometry[i].as_entire_binding(),
			}),
		});
		for &i in pass_indices(lighting) {
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

	pub fn copy_stats(&self, device: &wgpu::Device, encoder: &mut wgpu::CommandEncoder, lighting: LightingMode) -> FaceGiReadback {
		let buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("face_gi_stats"), size: u64::from(HEADER_WORDS) * 4,
			usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST, mapped_at_creation: false,
		}));
		encoder.copy_buffer_to_buffer(&self.words, 0, &buffer, 0, buffer.size());
		FaceGiReadback {
			buffer,
			stats: FaceGiStats {
				enabled: lighting != LightingMode::None, indirect: lighting == LightingMode::Indirect,
				rays_per_face: self.arena.params.settings[0], arena_bytes: self.arena.byte_size,
				visible_capacity: if lighting == LightingMode::Direct { self.arena.params.sizes[0] } else { self.arena.params.sizes[1] },
				table_capacity: self.arena.params.sizes[0],
				..Default::default()
			},
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	const RAY_COUNTS: [GiRayCount; 4] = [GiRayCount::Eight, GiRayCount::Sixteen, GiRayCount::ThirtyTwo, GiRayCount::SixtyFour];

	#[test]
	fn arena_ranges_are_contiguous_and_bounded() {
		let limits = wgpu::Limits::default();
		for ray_count in RAY_COUNTS {
			for (width, height) in [(1, 1), (9, 9), (1921, 1081), (3841, 2161)] {
				let layout = GiArenaLayout::new(width, height, ray_count, &limits).unwrap();
				let [table, visible, w, h] = layout.params.sizes;
				let [table_start, faces, hits, lighting] = layout.params.offsets;
				assert_eq!((w, h), (width, height));
				assert!(visible.is_power_of_two() && (MIN_VISIBLE..=MAX_VISIBLE).contains(&visible));
				assert_eq!(table, visible * 8);
				assert_eq!(table_start, HEADER_WORDS);
				assert_eq!(faces, table_start + table);
				assert_eq!(hits, faces + table);
				assert_eq!(lighting, hits + visible * ray_count.count() * 4);
				assert_eq!(layout.byte_size, u64::from(lighting + table * 7) * 4);
				assert_eq!(layout.clear_bytes, u64::from(faces) * 4);
				assert!(layout.clear_bytes < layout.byte_size);
				assert!(layout.byte_size <= limits.max_buffer_size.min(limits.max_storage_buffer_binding_size));
				let reference_end = u64::from(w) * u64::from(h) * 3 + u64::from(visible) * u64::from(ray_count.count());
				assert!(reference_end < u64::from(u32::MAX) - 1);
				let words: &[u32] = bytemuck::cast_slice(std::slice::from_ref(&layout.params));
				assert_eq!(words, &[table, visible, w, h, table_start, faces, hits, lighting,
					ray_count.count(), limits.max_compute_workgroups_per_dimension, 0, 0]);
			}
			assert_eq!(GiArenaLayout::new(1, 1, ray_count, &limits).unwrap().params.sizes[1], MIN_VISIBLE);
			assert_eq!(GiArenaLayout::new(9, 9, ray_count, &limits).unwrap().params.sizes[1], 128);
		}
		let layout = GiArenaLayout::new(1921, 1081, GiRayCount::Eight, &limits).unwrap();
		assert_eq!(layout.params.sizes[1], MAX_VISIBLE);
		assert_eq!(layout.byte_size, 104 * 1024 * 1024 + 64);
		assert_eq!(size_of::<GiParams>(), 48);
	}

	#[test]
	fn ray_count_sizes_hit_records_only() {
		for ray_count in RAY_COUNTS {
			let layout = GiArenaLayout::with_capacity(800, 600, 1024, ray_count, 192).unwrap();
			let baseline = GiArenaLayout::with_capacity(800, 600, 1024, GiRayCount::Eight, 192).unwrap();
			assert_eq!(layout.params.sizes, baseline.params.sizes);
			assert_eq!(layout.params.offsets[..3], baseline.params.offsets[..3]);
			assert_eq!(layout.clear_bytes, baseline.clear_bytes);
			assert_eq!(layout.byte_size - baseline.byte_size, u64::from(1024 * (ray_count.count() - 8) * 16));
		}
	}

	#[test]
	fn low_buffer_limits_reduce_capacity() {
		for ray_count in RAY_COUNTS {
			let target = GiArenaLayout::with_capacity(1024, 1024, 128, ray_count, 65535).unwrap();
			for storage_limit in [true, false] {
				let mut limits = wgpu::Limits::default();
				if storage_limit { limits.max_storage_buffer_binding_size = target.byte_size; }
				else { limits.max_buffer_size = target.byte_size; }
				let layout = GiArenaLayout::new(1024, 1024, ray_count, &limits).unwrap();
				assert_eq!(layout.params.sizes[1], 128);
				assert_eq!(layout.byte_size, target.byte_size);
			}
		}
	}

	#[test]
	fn larger_ray_counts_reduce_capacity_under_the_same_budget() {
		let mut limits = wgpu::Limits::default();
		limits.max_storage_buffer_binding_size = GiArenaLayout::with_capacity(1024, 1024, MAX_VISIBLE, GiRayCount::Eight, 65535).unwrap().byte_size;
		let capacities = RAY_COUNTS.map(|rays| GiArenaLayout::new(1024, 1024, rays, &limits).unwrap().params.sizes[1]);
		assert_eq!(capacities, [MAX_VISIBLE, MAX_VISIBLE / 2, MAX_VISIBLE / 2, MAX_VISIBLE / 4]);
	}

	#[test]
	fn impossible_limits_and_reference_overflow_are_rejected() {
		for ray_count in RAY_COUNTS {
			let mut limits = wgpu::Limits::default();
			limits.max_storage_buffer_binding_size = GiArenaLayout::with_capacity(1, 1, MIN_VISIBLE, ray_count, 65535).unwrap().byte_size - 1;
			assert!(GiArenaLayout::new(1, 1, ray_count, &limits).is_err());
			let mut limits = wgpu::Limits::default();
			limits.max_storage_buffers_per_shader_stage = 7;
			assert!(GiArenaLayout::new(1, 1, ray_count, &limits).is_err());
			let mut limits = wgpu::Limits::default();
			limits.max_bind_groups = 3;
			assert!(GiArenaLayout::new(1, 1, ray_count, &limits).is_err());
			let limits = wgpu::Limits::default();
			assert!(GiArenaLayout::new(0, 1, ray_count, &limits).is_err());
			assert!(GiArenaLayout::new(1, 0, ray_count, &limits).is_err());
			assert!(GiArenaLayout::new(65536, 65536, ray_count, &limits).is_err());
			assert!(GiArenaLayout::with_capacity(1, 1, u32::MAX, ray_count, 65535).is_err());
		}
	}

	#[test]
	fn low_dispatch_limits_use_two_dimensions_and_reduce_capacity() {
		let mut limits = wgpu::Limits::default();
		limits.max_compute_workgroups_per_dimension = 192;
		for ray_count in RAY_COUNTS {
			let layout = GiArenaLayout::new(800, 600, ray_count, &limits).unwrap();
			assert_eq!(layout.params.settings, [ray_count.count(), 192, 0, 0]);
			assert_eq!(layout.params.sizes[1], MAX_VISIBLE * 8 / ray_count.count());
			for count in [u64::from(layout.params.sizes[0]), u64::from(layout.params.sizes[1]) * u64::from(ray_count.count())] {
				let groups = count.div_ceil(64);
				let x = groups.min(192);
				let y = groups.div_ceil(x);
				assert!(y > 1 && y <= 192);
				assert!(x * y >= groups);
			}
		}
		assert!(GiArenaLayout::new(1537, 600, GiRayCount::Eight, &limits).is_err());
		assert!(GiArenaLayout::new(800, 769, GiRayCount::Eight, &limits).is_err());
		limits.max_compute_workgroups_per_dimension = 2;
		assert!(GiArenaLayout::new(1, 1, GiRayCount::Eight, &limits).is_err());
		limits.max_compute_workgroups_per_dimension = 0;
		assert!(GiArenaLayout::new(1, 1, GiRayCount::Eight, &limits).is_err());
	}

	#[test]
	fn dispatch_capacity_math_handles_large_limits() {
		let mut limits = wgpu::Limits::default();
		limits.max_compute_workgroups_per_dimension = u32::MAX;
		assert!(GiArenaLayout::new(800, 600, GiRayCount::SixtyFour, &limits).is_ok());
	}

	#[test]
	fn lighting_modes_schedule_only_required_passes() {
		assert!(pass_indices(LightingMode::None).is_empty());
		let entries = |mode| pass_indices(mode).iter().map(|&i| GI_ENTRIES[i]).collect::<Vec<_>>();
		assert_eq!(entries(LightingMode::Direct), ["discover_visible", "prepare_visible", "prepare_all", "light_direct"]);
		assert_eq!(entries(LightingMode::Indirect), GI_ENTRIES);
	}
}
