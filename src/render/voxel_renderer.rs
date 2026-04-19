use std::collections::HashMap;

use glam::IVec3;

use crate::{gpu_objects::{gpu_bvh, packed_dynamic_buffer::PackedDynamicBuffer}, physics::bvh, pose::Pose};

const BVH_BEAM_TEXTURE_FACTOR: u32 = 8;

pub struct VoxelRenderer {
	// Beam optimisation for BVH
	pub bvh_beam_textured: wgpu::Texture,
	pub bvh_beam_textured_storage_bind_group_layout: wgpu::BindGroupLayout,
	pub bvh_beam_textured_storage_bind_group: wgpu::BindGroup,
	pub bvh_beam_textured_read_bind_group_layout: wgpu::BindGroupLayout,
	pub bvh_beam_textured_read_bind_group: wgpu::BindGroup,
	// 64 tree
	pub packed_64_tree_dynamic_buffer: PackedDynamicBuffer,
	pub tree_bind_group_layout: wgpu::BindGroupLayout,
	// voxel data
	pub packed_voxel_data_dynamic_buffer: PackedDynamicBuffer,
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
	pub fn new(device: &wgpu::Device, config: &wgpu::SurfaceConfiguration, camera_bind_group_layout: &wgpu::BindGroupLayout) -> anyhow::Result<Self> {
		let packed_64_tree_dynamic_buffer = PackedDynamicBuffer::new(&device, 12, wgpu::BufferUsages::STORAGE);
		if let Err(err) = packed_64_tree_dynamic_buffer {
			println!("{}", err);
			return Err(anyhow::Error::msg(err));
		}
		let packed_64_tree_dynamic_buffer = packed_64_tree_dynamic_buffer.unwrap();
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

		let packed_voxel_data_dynamic_buffer = PackedDynamicBuffer::new(&device, 4, wgpu::BufferUsages::STORAGE);
		if let Err(err) = packed_voxel_data_dynamic_buffer {
			println!("{}", err);
			return Err(anyhow::Error::msg(err));
		}
		let packed_voxel_data_dynamic_buffer = packed_voxel_data_dynamic_buffer.unwrap();
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
					Some(&camera_bind_group_layout),
					Some(&gpu_bvh::GpuBvh::bind_group_layout(&device)),
					Some(&bvh_beam_textured_storage_bind_group_layout),
				],
				immediate_size: 0,
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
					Some(&camera_bind_group_layout),
					Some(&gpu_bvh::GpuBvh::bind_group_layout(&device)),
					Some(&tree_bind_group_layout),
					Some(&intermediate_textured_storage_bind_group_layout),
					Some(&bvh_beam_textured_read_bind_group_layout),
				],
				immediate_size: 0,
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
					Some(&camera_bind_group_layout),
					Some(&gpu_bvh::GpuBvh::bind_group_layout(&device)),
					Some(&voxel_bind_group_layout),
					Some(&intermediate_textured_read_bind_group_layout),
				],
				immediate_size: 0,
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
						format: config.format,
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
				multiview_mask: None,
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
			packed_64_tree_dynamic_buffer,
			tree_bind_group_layout,
			// voxel data
			packed_voxel_data_dynamic_buffer,
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

	pub fn resize(&mut self, device: &wgpu::Device, config: &wgpu::SurfaceConfiguration) {
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
		view: &wgpu::TextureView,
		camera_transform_bind_group: &wgpu::BindGroup,
		bvh: &bvh::BVH<(u32, u32, IVec3)>,
		gpu_grid_tree_id_to_id_poses: &HashMap<(u32, u32, IVec3), (u32, u32, Pose)>,
	) {
		let gpu_bvh = gpu_bvh::GpuBvh::from_bvh(&device, bvh, gpu_grid_tree_id_to_id_poses);
		{
			let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
				label: Some("Render Pass"),
				timestamp_writes: None,
			});

			compute_pass.set_bind_group(0, camera_transform_bind_group, &[]);
			compute_pass.set_bind_group(1, &gpu_bvh.bind_group, &[]);
			compute_pass.set_bind_group(2, &self.bvh_beam_textured_storage_bind_group, &[]);
			compute_pass.set_pipeline(&self.bvh_beam_pipeline);
			compute_pass.dispatch_workgroups((view.texture().width() / BVH_BEAM_TEXTURE_FACTOR + 7) / 4, (view.texture().height() / BVH_BEAM_TEXTURE_FACTOR + 3) / 4, 1);
		}
		{
			let tree_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &self.tree_bind_group_layout,
				entries: &[wgpu::BindGroupEntry {
					binding: 0,
					resource:  wgpu::BindingResource::Buffer(wgpu::BufferBinding {
						buffer: &self.packed_64_tree_dynamic_buffer.get_buffer(),
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
			compute_pass.dispatch_workgroups((view.texture().width() + 7) / 8, (view.texture().height() + 3) / 4, 1);
		}
		{
			let voxels_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &self.voxel_bind_group_layout,
				entries: &[wgpu::BindGroupEntry {
					binding: 0,
					resource:  wgpu::BindingResource::Buffer(wgpu::BufferBinding {
						buffer: &self.packed_voxel_data_dynamic_buffer.get_buffer(),
						offset: 0,
						size: None,
					}),
				}],
				label: Some("voxel_bind_group"),
			});
			let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
				label: Some("Render Pass"),
				color_attachments: &[Some(wgpu::RenderPassColorAttachment {
					view: &view,
					resolve_target: None,
					depth_slice: None,
					ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.1, g: 0.2, b: 0.3, a: 1.0 }), store: wgpu::StoreOp::Store },
				})],
				depth_stencil_attachment: None,
				occlusion_query_set: None,
				timestamp_writes: None,
				multiview_mask: None,
			});

			render_pass.set_bind_group(0, camera_transform_bind_group, &[]);
			render_pass.set_bind_group(1, &gpu_bvh.bind_group, &[]);
			render_pass.set_bind_group(2, &voxels_bind_group, &[]);
			render_pass.set_bind_group(3, &self.intermediate_textured_read_bind_group, &[]);
			render_pass.set_pipeline(&self.coloring_pipeline);
			render_pass.draw(0..3, 0..1);
		}
	}
}
