use std::collections::HashMap;

use voxel_data::{gpu_bvh::GpuBvh, bvh, grid::SubGridId};
use bevy::transform::components::Transform;
use bevy::ecs::entity::Entity;

pub const BVH_BEAM_TEXTURE_FACTOR: u32 = 8;

pub struct VoxelRenderer {
	// Beam optimisation for BVH
	pub bvh_beam_textured: wgpu::Texture,
	pub bvh_beam_textured_storage_bind_group_layout: wgpu::BindGroupLayout,
	pub bvh_beam_textured_storage_bind_group: wgpu::BindGroup,
	pub bvh_beam_textured_read_bind_group_layout: wgpu::BindGroupLayout,
	pub bvh_beam_textured_read_bind_group: wgpu::BindGroup,
	// 64 tree
	pub tree_bind_group_layout: wgpu::BindGroupLayout,
	// voxel data
	pub voxel_bind_group_layout: wgpu::BindGroupLayout,
	// intermediate textured
	pub intermediate_textured: wgpu::Texture,
	pub intermediate_textured_storage_bind_group_layout: wgpu::BindGroupLayout,
	pub intermediate_textured_storage_bind_group: wgpu::BindGroup,
	pub intermediate_textured_read_bind_group_layout: wgpu::BindGroupLayout,
	pub intermediate_textured_read_bind_group: wgpu::BindGroup,
	// pipelines
	pub bvh_beam_pipeline: wgpu::ComputePipeline,
	pub ray_marching_pipeline: wgpu::ComputePipeline,
	pub coloring_pipeline: wgpu::RenderPipeline,
}

impl VoxelRenderer {
	pub fn new(
		device: &wgpu::Device,
		width: u32,
		height: u32,
		color_format: wgpu::TextureFormat,
		camera_bind_group_layout: &wgpu::BindGroupLayout,
	) -> anyhow::Result<Self> {
		struct Cfg { width: u32, height: u32 }
		let config = Cfg { width, height };
		let tree_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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
		});

		let voxel_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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
		});

		// bvh beam optimisation
		let bvh_beam_textured_storage_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::COMPUTE,
				ty: wgpu::BindingType::StorageTexture { access:
					wgpu::StorageTextureAccess::WriteOnly, format:
					wgpu::TextureFormat::R32Float,
					view_dimension: wgpu::TextureViewDimension::D2,
				},
				count: None,
			}],
			label: Some("intermediate_textured_storage_bind_group_layout"),
		});
		let bvh_beam_textured_read_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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
			label: Some("intermediate_textured_read_bind_group_layout"),
		});
		let bvh_beam_textured = device.create_texture(&wgpu::TextureDescriptor {
			label: Some("intermediate_textured"),
			size: wgpu::Extent3d { width: config.width / BVH_BEAM_TEXTURE_FACTOR, height: config.height / BVH_BEAM_TEXTURE_FACTOR, depth_or_array_layers: 1 },
			mip_level_count: 1,
			sample_count: 1,
			dimension: wgpu::TextureDimension::D2,
			format: wgpu::TextureFormat::R32Float,
			usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::STORAGE_BINDING,
			view_formats: &[],
		});
		let bvh_beam_textured_storage_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &bvh_beam_textured_storage_bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource:  wgpu::BindingResource::TextureView(&bvh_beam_textured.create_view(&wgpu::TextureViewDescriptor {
					label: Some("intermediate_textured"),
					format: None,
					dimension: None,
					usage: Some(wgpu::TextureUsages::STORAGE_BINDING),
					aspect: wgpu::TextureAspect::All,
					base_mip_level: 0,
					mip_level_count: None,
					base_array_layer: 0,
					array_layer_count: None,
				})),
			}],
			label: Some("intermediate_textured_storage_bind_group"),
		});
		let bvh_beam_textured_read_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &bvh_beam_textured_read_bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource:  wgpu::BindingResource::TextureView(&bvh_beam_textured.create_view(&wgpu::TextureViewDescriptor {
					label: Some("intermediate_textured"),
					format: None,
					dimension: None,
					usage: Some(wgpu::TextureUsages::TEXTURE_BINDING),
					aspect: wgpu::TextureAspect::All,
					base_mip_level: 0,
					mip_level_count: None,
					base_array_layer: 0,
					array_layer_count: None,
				})),
			}],
			label: Some("intermediate_textured_read_bind_group"),
		});
		// bvh beam optimisation pipeline
		let bvh_beam_pipeline = {
			let bvh_beam_shader_src = concat!(
				include_str!("shaders/beam_bvh_raycast.wgsl"),
			);
			let bvh_beam_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
				label: Some("BVH Beam Shader"),
				source: wgpu::ShaderSource::Wgsl(bvh_beam_shader_src.into()),
			});
			let bvh_beam_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
				label: Some("BVH Beam Pipeline Layout"),
				bind_group_layouts: &[
					&camera_bind_group_layout,
					&GpuBvh::bind_group_layout(&device),
					&bvh_beam_textured_storage_bind_group_layout,
				],
				push_constant_ranges: &[],
			});
			device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
				label: Some("BVH Beam Pipeline"),
				layout: Some(&bvh_beam_pipeline_layout),
				module: &bvh_beam_shader,
				entry_point: Some("main"),
				compilation_options: Default::default(),
				cache: Default::default(),
			})
		};

		// intermediate textured
		let intermediate_textured_storage_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::COMPUTE,
				ty: wgpu::BindingType::StorageTexture { access:
					wgpu::StorageTextureAccess::WriteOnly, format:
					wgpu::TextureFormat::Rgba32Uint,
					view_dimension: wgpu::TextureViewDimension::D2,
				},
				count: None,
			}],
			label: Some("intermediate_textured_storage_bind_group_layout"),
		});
		let intermediate_textured_read_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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
			label: Some("intermediate_textured_read_bind_group_layout"),
		});
		let intermediate_textured = device.create_texture(&wgpu::TextureDescriptor {
			label: Some("intermediate_textured"),
			size: wgpu::Extent3d { width: config.width, height: config.height, depth_or_array_layers: 1 },
			mip_level_count: 1,
			sample_count: 1,
			dimension: wgpu::TextureDimension::D2,
			format: wgpu::TextureFormat::Rgba32Uint,
			usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::STORAGE_BINDING,
			view_formats: &[],
		});
		let intermediate_textured_storage_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &intermediate_textured_storage_bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource:  wgpu::BindingResource::TextureView(&intermediate_textured.create_view(&wgpu::TextureViewDescriptor {
					label: Some("intermediate_textured"),
					format: None,
					dimension: None,
					usage: Some(wgpu::TextureUsages::STORAGE_BINDING),
					aspect: wgpu::TextureAspect::All,
					base_mip_level: 0,
					mip_level_count: None,
					base_array_layer: 0,
					array_layer_count: None,
				})),
			}],
			label: Some("intermediate_textured_storage_bind_group"),
		});
		let intermediate_textured_read_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &intermediate_textured_read_bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource:  wgpu::BindingResource::TextureView(&intermediate_textured.create_view(&wgpu::TextureViewDescriptor {
					label: Some("intermediate_textured"),
					format: None,
					dimension: None,
					usage: Some(wgpu::TextureUsages::TEXTURE_BINDING),
					aspect: wgpu::TextureAspect::All,
					base_mip_level: 0,
					mip_level_count: None,
					base_array_layer: 0,
					array_layer_count: None,
				})),
			}],
			label: Some("intermediate_textured_read_bind_group"),
		});

		// ray marching
		let ray_marching_pipeline = {
			let ray_marching_shader_src = concat!(
				include_str!("shaders/bvh_raycast.wgsl"),
				include_str!("shaders/dda_raycast.wgsl"),
				include_str!("shaders/combined_raycast.wgsl"),
				include_str!("shaders/raycasting_shader.wgsl"),
			);
			let ray_marching_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
				label: Some("Ray Casting Shader"),
				source: wgpu::ShaderSource::Wgsl(ray_marching_shader_src.into()),
			});
			let ray_marching_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
				label: Some("Ray Casting Pipeline Layout"),
				bind_group_layouts: &[
					&camera_bind_group_layout,
					&GpuBvh::bind_group_layout(&device),
					&tree_bind_group_layout,
					&intermediate_textured_storage_bind_group_layout,
					&bvh_beam_textured_read_bind_group_layout,
				],
				push_constant_ranges: &[],
			});
			device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
				label: Some("Ray Casting Pipeline"),
				layout: Some(&ray_marching_pipeline_layout),
				module: &ray_marching_shader,
				entry_point: Some("main"),
				compilation_options: Default::default(),
				cache: Default::default(),
			})
		};

		// coloring
		let coloring_pipeline = {
			let coloring_shader_src = concat!(
				include_str!("shaders/bvh_raycast.wgsl"),
				include_str!("shaders/voxel_reader.wgsl"),
				include_str!("shaders/coloring_shader.wgsl"),
			);
			let coloring_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
				label: Some("Coloring Shader"),
				source: wgpu::ShaderSource::Wgsl(coloring_shader_src.into()),
			});
			let coloring_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
				label: Some("Coloring Pipeline Layout"),
				bind_group_layouts: &[
					&camera_bind_group_layout,
					&GpuBvh::bind_group_layout(&device),
					&voxel_bind_group_layout,
					&intermediate_textured_read_bind_group_layout,
				],
				push_constant_ranges: &[],
			});
			device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
				label: Some("Coloring Pipeline"),
				layout: Some(&coloring_pipeline_layout),
				vertex: wgpu::VertexState {
					module: &coloring_shader,
					entry_point: Some("vs_main"),
					buffers: &[],
					compilation_options: wgpu::PipelineCompilationOptions::default(),
				},
				fragment: Some(wgpu::FragmentState {
					module: &coloring_shader,
					entry_point: Some("fs_main"),
					targets: &[Some(wgpu::ColorTargetState {
						format: color_format,
						blend: Some(wgpu::BlendState::REPLACE),
						write_mask: wgpu::ColorWrites::ALL,
					})],
					compilation_options: wgpu::PipelineCompilationOptions::default(),
				}),
				primitive: wgpu::PrimitiveState {
					topology: wgpu::PrimitiveTopology::TriangleList, // 1.
					strip_index_format: None,
					front_face: wgpu::FrontFace::Ccw, // 2.
					cull_mode: Some(wgpu::Face::Back),
					// Setting this to anything other than Fill requires Features::NON_FILL_POLYGON_MODE
					polygon_mode: wgpu::PolygonMode::Fill,
					// Requires Features::DEPTH_CLIP_CONTROL
					unclipped_depth: false,
					// Requires Features::CONSERVATIVE_RASTERIZATION
					conservative: false,
				},
				depth_stencil: None,
				multisample: wgpu::MultisampleState {
					count: 1,
					mask: !0,
					alpha_to_coverage_enabled: false,
				},
				multiview: None,
				cache: None,
			})
		};

		Ok(Self {
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
			intermediate_textured_storage_bind_group_layout,
			intermediate_textured_storage_bind_group,
			intermediate_textured_read_bind_group_layout,
			intermediate_textured_read_bind_group,
			// pipelines
			ray_marching_pipeline,
			coloring_pipeline,
		})
	}

	pub fn resize(&mut self, device: &wgpu::Device, width: u32, height: u32) {
		struct Cfg { width: u32, height: u32 }
		let config = Cfg { width, height };
		self.bvh_beam_textured = device.create_texture(&wgpu::TextureDescriptor {
			label: Some("intermediate_textured"),
			size: wgpu::Extent3d { width: config.width / BVH_BEAM_TEXTURE_FACTOR, height: config.height / BVH_BEAM_TEXTURE_FACTOR, depth_or_array_layers: 1 },
			mip_level_count: 1,
			sample_count: 1,
			dimension: wgpu::TextureDimension::D2,
			format: wgpu::TextureFormat::R32Float,
			usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::STORAGE_BINDING,
			view_formats: &[],
		});
		self.bvh_beam_textured_storage_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &self.bvh_beam_textured_storage_bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource:  wgpu::BindingResource::TextureView(&self.bvh_beam_textured.create_view(&wgpu::TextureViewDescriptor {
					label: Some("intermediate_textured"),
					format: None,
					dimension: None,
					usage: Some(wgpu::TextureUsages::STORAGE_BINDING),
					aspect: wgpu::TextureAspect::All,
					base_mip_level: 0,
					mip_level_count: None,
					base_array_layer: 0,
					array_layer_count: None,
				})),
			}],
			label: Some("intermediate_textured_storage_bind_group"),
		});
		self.bvh_beam_textured_read_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &self.bvh_beam_textured_read_bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource:  wgpu::BindingResource::TextureView(&self.bvh_beam_textured.create_view(&wgpu::TextureViewDescriptor {
					label: Some("intermediate_textured"),
					format: None,
					dimension: None,
					usage: Some(wgpu::TextureUsages::TEXTURE_BINDING),
					aspect: wgpu::TextureAspect::All,
					base_mip_level: 0,
					mip_level_count: None,
					base_array_layer: 0,
					array_layer_count: None,
				})),
			}],
			label: Some("intermediate_textured_read_bind_group"),
		});

		self.intermediate_textured = device.create_texture(&wgpu::TextureDescriptor {
			label: Some("intermediate_textured"),
			size: wgpu::Extent3d { width: config.width, height: config.height, depth_or_array_layers: 1 },
			mip_level_count: 1,
			sample_count: 1,
			dimension: wgpu::TextureDimension::D2,
			format: wgpu::TextureFormat::Rgba32Uint,
			usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::STORAGE_BINDING,
			view_formats: &[],
		});
		self.intermediate_textured_storage_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &self.intermediate_textured_storage_bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource:  wgpu::BindingResource::TextureView(&self.intermediate_textured.create_view(&wgpu::TextureViewDescriptor {
					label: Some("intermediate_textured"),
					format: None,
					dimension: None,
					usage: Some(wgpu::TextureUsages::STORAGE_BINDING),
					aspect: wgpu::TextureAspect::All,
					base_mip_level: 0,
					mip_level_count: None,
					base_array_layer: 0,
					array_layer_count: None,
				})),
			}],
			label: Some("intermediate_textured_storage_bind_group"),
		});
		self.intermediate_textured_read_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &self.intermediate_textured_read_bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource:  wgpu::BindingResource::TextureView(&self.intermediate_textured.create_view(&wgpu::TextureViewDescriptor {
					label: Some("intermediate_textured"),
					format: None,
					dimension: None,
					usage: Some(wgpu::TextureUsages::TEXTURE_BINDING),
					aspect: wgpu::TextureAspect::All,
					base_mip_level: 0,
					mip_level_count: None,
					base_array_layer: 0,
					array_layer_count: None,
				})),
			}],
			label: Some("intermediate_textured_read_bind_group"),
		});
	}

	pub fn render(
		&self,
		device: &wgpu::Device,
		encoder: &mut wgpu::CommandEncoder,
		view_width: u32,
		view_height: u32,
		camera_transform_bind_group: &wgpu::BindGroup,
		bvh: &bvh::BVH<(Entity, SubGridId)>,
		gpu_grid_tree_id_to_id_transforms: &HashMap<(Entity, SubGridId), (u32, u32, Transform)>,
		tree_buffer: &wgpu::Buffer,
		voxel_buffer: &wgpu::Buffer,
		color_attachment: wgpu::RenderPassColorAttachment<'_>,
	) -> GpuBvh {
		let gpu_bvh = GpuBvh::from_bvh(&device, bvh, gpu_grid_tree_id_to_id_transforms);
		{
			let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
				label: Some("Render Pass"),
				timestamp_writes: None,
			});

			compute_pass.set_bind_group(0, camera_transform_bind_group, &[]);
			compute_pass.set_bind_group(1, &gpu_bvh.bind_group, &[]);
			compute_pass.set_bind_group(2, &self.bvh_beam_textured_storage_bind_group, &[]);
			compute_pass.set_pipeline(&self.bvh_beam_pipeline);
			compute_pass.dispatch_workgroups((view_width / BVH_BEAM_TEXTURE_FACTOR + 7) / 4, (view_height / BVH_BEAM_TEXTURE_FACTOR + 3) / 4, 1);
		}
		{
			let tree_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &self.tree_bind_group_layout,
				entries: &[wgpu::BindGroupEntry {
					binding: 0,
					resource:  wgpu::BindingResource::Buffer(wgpu::BufferBinding {
						buffer: tree_buffer,
						offset: 0,
						size: None,
					}),
				}],
				label: Some("tree_bind_group"),
			});
			let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
				label: Some("Render Pass"),
				timestamp_writes: None,
			});

			compute_pass.set_bind_group(0, camera_transform_bind_group, &[]);
			compute_pass.set_bind_group(1, &gpu_bvh.bind_group, &[]);
			compute_pass.set_bind_group(2, &tree_bind_group, &[]);
			compute_pass.set_bind_group(3, &self.intermediate_textured_storage_bind_group, &[]);
			compute_pass.set_bind_group(4, &self.bvh_beam_textured_read_bind_group, &[]);
			compute_pass.set_pipeline(&self.ray_marching_pipeline);
			compute_pass.dispatch_workgroups((view_width + 7) / 8, (view_height + 3) / 4, 1);
		}
		{
			let voxels_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &self.voxel_bind_group_layout,
				entries: &[wgpu::BindGroupEntry {
					binding: 0,
					resource:  wgpu::BindingResource::Buffer(wgpu::BufferBinding {
						buffer: voxel_buffer,
						offset: 0,
						size: None,
					}),
				}],
				label: Some("voxel_bind_group"),
			});
			let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
				label: Some("Voxel Coloring Pass"),
				color_attachments: &[Some(color_attachment)],
				depth_stencil_attachment: None,
				occlusion_query_set: None,
				timestamp_writes: None,
			});

			render_pass.set_bind_group(0, camera_transform_bind_group, &[]);
			render_pass.set_bind_group(1, &gpu_bvh.bind_group, &[]);
			render_pass.set_bind_group(2, &voxels_bind_group, &[]);
			render_pass.set_bind_group(3, &self.intermediate_textured_read_bind_group, &[]);
			render_pass.set_pipeline(&self.coloring_pipeline);
			render_pass.draw(0..3, 0..1);
		}

		encoder.copy_buffer_to_buffer(
			&gpu_bvh.item_hit_count_buffer, 0,
			&gpu_bvh.item_hit_count_staging_buffer, 0,
			gpu_bvh.item_hit_count_buffer.size(),
		);

		gpu_bvh
	}
}

use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::RenderDevice;

#[derive(Resource)]
pub struct VoxelRendererResource {
	pub voxel_renderer: Option<VoxelRenderer>,
	pub size: (u32, u32),
	pub format: Option<wgpu::TextureFormat>,
	pub camera_buffer: wgpu::Buffer,
	pub render_settings_buffer: wgpu::Buffer,
	pub camera_bind_group: wgpu::BindGroup,
	pub camera_bind_group_layout: wgpu::BindGroupLayout,
}

impl FromWorld for VoxelRendererResource {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let device = render_device.wgpu_device();

		let camera_buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("voxel_camera_uniform"),
			size: std::mem::size_of::<crate::camera::CameraUniform>() as u64,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		});
		let render_settings_buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("voxel_render_settings_uniform"),
			size: std::mem::size_of::<crate::graphics_settings::RenderSettingsUniform>() as u64,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		});

		let camera_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[
				wgpu::BindGroupLayoutEntry {
					binding: 0,
					visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: None,
					},
					count: None,
				},
				wgpu::BindGroupLayoutEntry {
					binding: 1,
					visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: None,
					},
					count: None,
				},
			],
			label: Some("voxel_camera_bind_group_layout"),
		});
		let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &camera_bind_group_layout,
			entries: &[
				wgpu::BindGroupEntry { binding: 0, resource: camera_buffer.as_entire_binding() },
				wgpu::BindGroupEntry { binding: 1, resource: render_settings_buffer.as_entire_binding() },
			],
			label: Some("voxel_camera_bind_group"),
		});

		Self {
			voxel_renderer: None,
			size: (0, 0),
			format: None,
			camera_buffer,
			render_settings_buffer,
			camera_bind_group,
			camera_bind_group_layout,
		}
	}
}

impl VoxelRendererResource {
	pub fn ensure(&mut self, device: &wgpu::Device, width: u32, height: u32, format: wgpu::TextureFormat) {
		if width == 0 || height == 0 { return; }

		let need_rebuild = self.voxel_renderer.is_none() || self.format != Some(format);
		if need_rebuild {
			match VoxelRenderer::new(device, width, height, format, &self.camera_bind_group_layout) {
				Ok(vr) => {
					self.voxel_renderer = Some(vr);
					self.size = (width, height);
					self.format = Some(format);
				},
				Err(e) => {
					log::error!("Failed to build VoxelRenderer: {e}");
				}
			}
			return;
		}

		if self.size != (width, height) {
			if let Some(vr) = self.voxel_renderer.as_mut() {
				vr.resize(device, width, height);
				self.size = (width, height);
			}
		}
	}
}
