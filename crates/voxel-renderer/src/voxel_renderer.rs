use std::collections::HashMap;

use bevy::ecs::entity::Entity;
use bevy::render::renderer::WgpuWrapper;
use bevy::transform::components::Transform;
use voxel_data::bvh;

type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBindGroupLayout = WgpuWrapper<wgpu::BindGroupLayout>;
type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuComputePipeline = WgpuWrapper<wgpu::ComputePipeline>;
type GpuRenderPipeline = WgpuWrapper<wgpu::RenderPipeline>;
type GpuTexture = WgpuWrapper<wgpu::Texture>;

use crate::gpu_bvh::GpuBvh;

pub const BVH_BEAM_TEXTURE_FACTOR: u32 = 8;

pub struct VoxelRenderer {
	pub color_format: wgpu::TextureFormat,
	pub camera_bind_group_layout: GpuBindGroupLayout,
	// Beam optimisation for BVH
	pub bvh_beam_textured: GpuTexture,
	pub bvh_beam_textured_storage_bind_group_layout: GpuBindGroupLayout,
	pub bvh_beam_textured_storage_bind_group: GpuBindGroup,
	pub bvh_beam_textured_read_bind_group_layout: GpuBindGroupLayout,
	pub bvh_beam_textured_read_bind_group: GpuBindGroup,
	// 64 tree
	pub tree_bind_group_layout: GpuBindGroupLayout,
	// voxel data
	pub voxel_bind_group_layout: GpuBindGroupLayout,
	// intermediate textured
	pub intermediate_textured: GpuTexture,
	pub ray_marching_bind_group_layout: GpuBindGroupLayout,
	pub ray_marching_bind_group: GpuBindGroup,
	pub intermediate_textured_read_bind_group_layout: GpuBindGroupLayout,
	pub intermediate_textured_read_bind_group: GpuBindGroup,
	// pipelines
	pub bvh_beam_pipeline: GpuComputePipeline,
	pub ray_marching_pipeline: GpuComputePipeline,
	pub coloring_pipeline: GpuRenderPipeline,
}

fn texture_view(_texture: &GpuTexture, usage: wgpu::TextureUsages) -> impl FnOnce(&wgpu::Texture) -> wgpu::TextureView + '_ {
	move |texture| {
		texture.create_view(&wgpu::TextureViewDescriptor {
			label: Some("texture_view"),
			format: None,
			dimension: None,
			usage: Some(usage),
			aspect: wgpu::TextureAspect::All,
			base_mip_level: 0,
			mip_level_count: None,
			base_array_layer: 0,
			array_layer_count: None,
		})
	}
}

impl VoxelRenderer {
	pub fn new(
		device: &wgpu::Device,
		width: u32,
		height: u32,
		color_format: wgpu::TextureFormat,
		camera_bind_group_layout: &GpuBindGroupLayout,
	) -> anyhow::Result<Self> {
		struct Cfg { width: u32, height: u32 }
		let config = Cfg { width, height };
		let tree_bind_group_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::COMPUTE,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Storage { read_only: true },
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
			label: Some("tree_bind_group_layout"),
		}));
		let voxel_bind_group_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Storage { read_only: true },
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
			label: Some("voxels_bind_group_layout"),
		}));

		let bvh_beam_textured_storage_bind_group_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::COMPUTE,
				ty: wgpu::BindingType::StorageTexture {
					access: wgpu::StorageTextureAccess::WriteOnly,
					format: wgpu::TextureFormat::R32Float,
					view_dimension: wgpu::TextureViewDimension::D2,
				},
				count: None,
			}],
			label: Some("bvh_beam_storage_layout"),
		}));
		let bvh_beam_textured_read_bind_group_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::COMPUTE,
				ty: wgpu::BindingType::Texture {
					sample_type: wgpu::TextureSampleType::Float { filterable: false },
					view_dimension: wgpu::TextureViewDimension::D2,
					multisampled: false,
				},
				count: None,
			}],
			label: Some("bvh_beam_read_layout"),
		}));
		let bvh_beam_textured = WgpuWrapper::new(device.create_texture(&wgpu::TextureDescriptor {
			label: Some("bvh_beam_texture"),
			size: wgpu::Extent3d { width: config.width / BVH_BEAM_TEXTURE_FACTOR, height: config.height / BVH_BEAM_TEXTURE_FACTOR, depth_or_array_layers: 1 },
			mip_level_count: 1,
			sample_count: 1,
			dimension: wgpu::TextureDimension::D2,
			format: wgpu::TextureFormat::R32Float,
			usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::STORAGE_BINDING,
			view_formats: &[],
		}));
		let bvh_beam_textured_storage_bind_group = {
			let view = texture_view(&bvh_beam_textured, wgpu::TextureUsages::STORAGE_BINDING)(&bvh_beam_textured);
			WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &bvh_beam_textured_storage_bind_group_layout,
				entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&view) }],
				label: Some("bvh_beam_storage_bind_group"),
			}))
		};
		let bvh_beam_textured_read_bind_group = {
			let view = texture_view(&bvh_beam_textured, wgpu::TextureUsages::TEXTURE_BINDING)(&bvh_beam_textured);
			WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &bvh_beam_textured_read_bind_group_layout,
				entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&view) }],
				label: Some("bvh_beam_read_bind_group"),
			}))
		};

		let ray_marching_bind_group_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[
				wgpu::BindGroupLayoutEntry {
					binding: 0,
					visibility: wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::StorageTexture {
						access: wgpu::StorageTextureAccess::WriteOnly,
						format: wgpu::TextureFormat::Rgba32Uint,
						view_dimension: wgpu::TextureViewDimension::D2,
					},
					count: None,
				},
				wgpu::BindGroupLayoutEntry {
					binding: 1,
					visibility: wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Texture {
						sample_type: wgpu::TextureSampleType::Float { filterable: false },
						view_dimension: wgpu::TextureViewDimension::D2,
						multisampled: false,
					},
					count: None,
				},
			],
			label: Some("ray_marching_bind_group_layout"),
		}));
		let intermediate_textured_read_bind_group_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Texture {
					sample_type: wgpu::TextureSampleType::Uint,
					view_dimension: wgpu::TextureViewDimension::D2,
					multisampled: false,
				},
				count: None,
			}],
			label: Some("intermediate_read_layout"),
		}));
		let intermediate_textured = WgpuWrapper::new(device.create_texture(&wgpu::TextureDescriptor {
			label: Some("intermediate_texture"),
			size: wgpu::Extent3d { width: config.width, height: config.height, depth_or_array_layers: 1 },
			mip_level_count: 1,
			sample_count: 1,
			dimension: wgpu::TextureDimension::D2,
			format: wgpu::TextureFormat::Rgba32Uint,
			usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::STORAGE_BINDING,
			view_formats: &[],
		}));
		let ray_marching_bind_group = {
			let intermediate_view = texture_view(&intermediate_textured, wgpu::TextureUsages::STORAGE_BINDING)(&intermediate_textured);
			let beam_view = texture_view(&bvh_beam_textured, wgpu::TextureUsages::TEXTURE_BINDING)(&bvh_beam_textured);
			WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &ray_marching_bind_group_layout,
				entries: &[
					wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&intermediate_view) },
					wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::TextureView(&beam_view) },
				],
				label: Some("ray_marching_bind_group"),
			}))
		};
		let intermediate_textured_read_bind_group = {
			let view = texture_view(&intermediate_textured, wgpu::TextureUsages::TEXTURE_BINDING)(&intermediate_textured);
			WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &intermediate_textured_read_bind_group_layout,
				entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&view) }],
				label: Some("intermediate_read_bind_group"),
			}))
		};

		let gpu_bvh_layout = GpuBvh::<Entity>::bind_group_layout(device);
		let bvh_beam_pipeline = {
			let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
				label: Some("Beam Shader"),
				source: wgpu::ShaderSource::Wgsl(include_str!(concat!(env!("OUT_DIR"), "/beam.wgsl")).into()),
			});
			let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
				label: Some("Beam Pipeline Layout"),
				bind_group_layouts: &[camera_bind_group_layout, &gpu_bvh_layout, &tree_bind_group_layout, &bvh_beam_textured_storage_bind_group_layout],
				push_constant_ranges: &[],
			});
			WgpuWrapper::new(device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
				label: Some("Beam Pipeline"),
				layout: Some(&layout),
				module: &shader,
				entry_point: Some("main"),
				compilation_options: Default::default(),
				cache: Default::default(),
			}))
		};
		let ray_marching_pipeline = {
			let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
				label: Some("Ray Casting Shader"),
				source: wgpu::ShaderSource::Wgsl(include_str!(concat!(env!("OUT_DIR"), "/raycasting.wgsl")).into()),
			});
			let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
				label: Some("Ray Casting Pipeline Layout"),
				bind_group_layouts: &[camera_bind_group_layout, &gpu_bvh_layout, &tree_bind_group_layout, &ray_marching_bind_group_layout],
				push_constant_ranges: &[],
			});
			WgpuWrapper::new(device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
				label: Some("Ray Casting Pipeline"),
				layout: Some(&layout),
				module: &shader,
				entry_point: Some("main"),
				compilation_options: Default::default(),
				cache: Default::default(),
			}))
		};

		// coloring
		let coloring_pipeline = {
			let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
				label: Some("Coloring Shader"),
				source: wgpu::ShaderSource::Wgsl(include_str!(concat!(env!("OUT_DIR"), "/coloring_shader.wgsl")).into()),
			});
			let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
				label: Some("Coloring Pipeline Layout"),
				bind_group_layouts: &[camera_bind_group_layout, &gpu_bvh_layout, &voxel_bind_group_layout, &intermediate_textured_read_bind_group_layout],
				push_constant_ranges: &[],
			});
			WgpuWrapper::new(device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
				label: Some("Coloring Pipeline"),
				layout: Some(&layout),
				vertex: wgpu::VertexState {
					module: &shader,
					entry_point: Some("vs_main"),
					buffers: &[],
					compilation_options: wgpu::PipelineCompilationOptions::default(),
				},
				fragment: Some(wgpu::FragmentState {
					module: &shader,
					entry_point: Some("fs_main"),
					targets: &[Some(wgpu::ColorTargetState { format: color_format, blend: None, write_mask: wgpu::ColorWrites::ALL })],
					compilation_options: wgpu::PipelineCompilationOptions::default(),
				}),
				primitive: wgpu::PrimitiveState { topology: wgpu::PrimitiveTopology::TriangleList, strip_index_format: None, front_face: wgpu::FrontFace::Ccw, cull_mode: Some(wgpu::Face::Back), polygon_mode: wgpu::PolygonMode::Fill, unclipped_depth: false, conservative: false },
				depth_stencil: None,
				multisample: wgpu::MultisampleState { count: 1, mask: !0, alpha_to_coverage_enabled: false },
				multiview: None,
				cache: None,
			}))
		};

		Ok(Self {
			color_format,
			camera_bind_group_layout: camera_bind_group_layout.clone(),
			// bvh beam optimization
			bvh_beam_textured_storage_bind_group_layout,
			bvh_beam_textured_read_bind_group_layout,
			bvh_beam_textured,
			bvh_beam_textured_storage_bind_group,
			bvh_beam_textured_read_bind_group,
			bvh_beam_pipeline,
			// 64 tree
			tree_bind_group_layout,
			// voxel data
			voxel_bind_group_layout,
			// intermediate textured
			intermediate_textured,
			ray_marching_bind_group_layout,
			ray_marching_bind_group,
			intermediate_textured_read_bind_group_layout,
			intermediate_textured_read_bind_group,
			// pipelines
			ray_marching_pipeline,
			coloring_pipeline,
		})
	}

	pub fn resize(&mut self, device: &wgpu::Device, width: u32, height: u32) {
		let rebuilt = Self::new(device, width, height, self.color_format, &self.camera_bind_group_layout)
			.expect("Failed to resize VoxelRenderer");
		self.bvh_beam_textured = rebuilt.bvh_beam_textured;
		self.bvh_beam_textured_storage_bind_group_layout = rebuilt.bvh_beam_textured_storage_bind_group_layout;
		self.bvh_beam_textured_storage_bind_group = rebuilt.bvh_beam_textured_storage_bind_group;
		self.bvh_beam_textured_read_bind_group_layout = rebuilt.bvh_beam_textured_read_bind_group_layout;
		self.bvh_beam_textured_read_bind_group = rebuilt.bvh_beam_textured_read_bind_group;
		self.tree_bind_group_layout = rebuilt.tree_bind_group_layout;
		self.voxel_bind_group_layout = rebuilt.voxel_bind_group_layout;
		self.intermediate_textured = rebuilt.intermediate_textured;
		self.ray_marching_bind_group_layout = rebuilt.ray_marching_bind_group_layout;
		self.ray_marching_bind_group = rebuilt.ray_marching_bind_group;
		self.intermediate_textured_read_bind_group_layout = rebuilt.intermediate_textured_read_bind_group_layout;
		self.intermediate_textured_read_bind_group = rebuilt.intermediate_textured_read_bind_group;
		self.bvh_beam_pipeline = rebuilt.bvh_beam_pipeline;
		self.ray_marching_pipeline = rebuilt.ray_marching_pipeline;
		self.coloring_pipeline = rebuilt.coloring_pipeline;
	}

	pub fn render(
		&self,
		device: &wgpu::Device,
		encoder: &mut wgpu::CommandEncoder,
		view_width: u32,
		view_height: u32,
		camera_transform_bind_group: &GpuBindGroup,
		bvh: &bvh::BVH<Entity>,
		gpu_grid_tree_id_to_id_transforms: &HashMap<Entity, (u32, u32, Transform)>,
		tree_buffer: &GpuBuffer,
		voxel_buffer: &GpuBuffer,
		color_attachment: wgpu::RenderPassColorAttachment<'_>,
	) -> GpuBvh<Entity> {
		let gpu_bvh = GpuBvh::from_bvh(device, bvh, gpu_grid_tree_id_to_id_transforms);
		let tree_bind_group = WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &self.tree_bind_group_layout,
			entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding { buffer: tree_buffer, offset: 0, size: None }) }],
			label: Some("tree_bind_group"),
		}));

		{
			let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor { label: Some("Beam Pass"), timestamp_writes: None });
			compute_pass.set_bind_group(0, &**camera_transform_bind_group, &[]);
			compute_pass.set_bind_group(1, &*gpu_bvh.bind_group, &[]);
			compute_pass.set_bind_group(2, &*tree_bind_group, &[]);
			compute_pass.set_bind_group(3, &*self.bvh_beam_textured_storage_bind_group, &[]);
			compute_pass.set_pipeline(&self.bvh_beam_pipeline);
			compute_pass.dispatch_workgroups((view_width / BVH_BEAM_TEXTURE_FACTOR + 7) / 4, (view_height / BVH_BEAM_TEXTURE_FACTOR + 3) / 4, 1);
		}
		{
			let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor { label: Some("Ray Pass"), timestamp_writes: None });
			compute_pass.set_bind_group(0, &**camera_transform_bind_group, &[]);
			compute_pass.set_bind_group(1, &*gpu_bvh.bind_group, &[]);
			compute_pass.set_bind_group(2, &*tree_bind_group, &[]);
			compute_pass.set_bind_group(3, &*self.ray_marching_bind_group, &[]);
			compute_pass.set_pipeline(&self.ray_marching_pipeline);
			compute_pass.dispatch_workgroups((view_width + 7) / 8, (view_height + 3) / 4, 1);
		}
		let voxels_bind_group = WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &self.voxel_bind_group_layout,
			entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding { buffer: voxel_buffer, offset: 0, size: None }) }],
			label: Some("voxel_bind_group"),
		}));
		{
			let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
				label: Some("Voxel Coloring Pass"),
				color_attachments: &[Some(color_attachment)],
				depth_stencil_attachment: None,
				occlusion_query_set: None,
				timestamp_writes: None,
			});
			render_pass.set_bind_group(0, &**camera_transform_bind_group, &[]);
			render_pass.set_bind_group(1, &*gpu_bvh.bind_group, &[]);
			render_pass.set_bind_group(2, &*voxels_bind_group, &[]);
			render_pass.set_bind_group(3, &*self.intermediate_textured_read_bind_group, &[]);
			render_pass.set_pipeline(&self.coloring_pipeline);
			render_pass.draw(0..3, 0..1);
		}

		encoder.copy_buffer_to_buffer(
			&gpu_bvh.item_hit_count_buffer,
			0,
			&gpu_bvh.item_hit_count_staging_buffer,
			0,
			gpu_bvh.item_hit_count_buffer.size(),
		);

		gpu_bvh
	}
}
