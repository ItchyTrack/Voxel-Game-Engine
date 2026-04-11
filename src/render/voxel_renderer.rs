use std::collections::HashMap;

use glam::IVec3;

use crate::{gpu_objects::{gpu_bvh, packed_dynamic_buffer::PackedDynamicBuffer}, physics::bvh, pose::Pose};

pub struct VoxelRenderer {
	pub packed_64_tree_dynamic_buffer: PackedDynamicBuffer,
	pub packed_voxel_data_dynamic_buffer: PackedDynamicBuffer,
	pub ray_marching_pipeline: wgpu::RenderPipeline,
}

impl VoxelRenderer {
	pub fn new(device: &wgpu::Device, config: &wgpu::SurfaceConfiguration, camera_bind_group_layout: &wgpu::BindGroupLayout) -> anyhow::Result<Self> {
		// let depth_texture = texture::Texture::create_depth_texture(&device, &config, "depth_texture");

		let tree_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Storage { read_only: true },
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			},wgpu::BindGroupLayoutEntry {
				binding: 1,
				visibility: wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Storage { read_only: true },
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
			label: Some("gtree_bind_group_layout"),
		});
		let shader_src = concat!(
			include_str!("shaders/bvh_raycast.wgsl"),
			include_str!("shaders/voxel_reader.wgsl"),
			include_str!("shaders/dda_raycast.wgsl"),
			include_str!("shaders/combined_raycast.wgsl"),
			include_str!("shaders/main_shader.wgsl"),
		);
		let ray_marching_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("Ray Casting Shader"),
			source: wgpu::ShaderSource::Wgsl(shader_src.into()),
		});
		let ray_marching_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: Some("Ray Casting Pipeline Layout"),
			bind_group_layouts: &[
				Some(&camera_bind_group_layout),
				Some(&gpu_bvh::GpuBvh::bind_group_layout(&device)),
				Some(&tree_bind_group_layout),
			],
			immediate_size: 0,
		});
		let ray_marching_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
			label: Some("Ray Casting Pipeline"),
			layout: Some(&ray_marching_pipeline_layout),
			vertex: wgpu::VertexState {
				module: &ray_marching_shader,
				entry_point: Some("vs_main"),
				buffers: &[],
				compilation_options: wgpu::PipelineCompilationOptions::default(),
			},
			fragment: Some(wgpu::FragmentState {
				module: &ray_marching_shader,
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
			depth_stencil: None, //Some(wgpu::DepthStencilState {
			// 	format: texture::Texture::DEPTH_FORMAT,
			// 	depth_write_enabled: Some(true),
			// 	depth_compare: Some(wgpu::CompareFunction::Less),
			// 	stencil: wgpu::StencilState::default(),
			// 	bias: wgpu::DepthBiasState::default(),
			// }),
			multisample: wgpu::MultisampleState {
				count: 1,
				mask: !0,
				alpha_to_coverage_enabled: false,
			},
			multiview_mask: None,
			cache: None,
		});

		let packed_64_tree_dynamic_buffer = PackedDynamicBuffer::new(&device, 12, wgpu::BufferUsages::STORAGE);
		if let Err(err) = packed_64_tree_dynamic_buffer {
			println!("{}", err);
			return Err(anyhow::Error::msg(err));
		}
		let packed_voxel_data_dynamic_buffer = PackedDynamicBuffer::new(&device, 4, wgpu::BufferUsages::STORAGE);
		if let Err(err) = packed_voxel_data_dynamic_buffer {
			println!("{}", err);
			return Err(anyhow::Error::msg(err));
		}
		Ok(Self {
			packed_64_tree_dynamic_buffer: packed_64_tree_dynamic_buffer.unwrap(),
			packed_voxel_data_dynamic_buffer: packed_voxel_data_dynamic_buffer.unwrap(),
			ray_marching_pipeline: ray_marching_pipeline,
		})
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
		let tree_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Storage { read_only: true },
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			},wgpu::BindGroupLayoutEntry {
				binding: 1,
				visibility: wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Storage { read_only: true },
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
			label: Some("tree_bind_group_layout"),
		});
		let tree_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &tree_bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource:  wgpu::BindingResource::Buffer(wgpu::BufferBinding {
					buffer: &self.packed_64_tree_dynamic_buffer.get_buffer(),
					offset: 0,
					size: None,
				}),
			},wgpu::BindGroupEntry {
				binding: 1,
				resource:  wgpu::BindingResource::Buffer(wgpu::BufferBinding {
					buffer: &self.packed_voxel_data_dynamic_buffer.get_buffer(),
					offset: 0,
					size: None,
				}),
			}],
			label: Some("tree_bind_group"),
		});

		let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
			label: Some("Render Pass"),
			color_attachments: &[Some(wgpu::RenderPassColorAttachment {
				view: &view,
				resolve_target: None,
				depth_slice: None,
				ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.1, g: 0.2, b: 0.3, a: 1.0 }), store: wgpu::StoreOp::Store },
			})],
			depth_stencil_attachment: None, //Some(wgpu::RenderPassDepthStencilAttachment {
			// 	view: &self.depth_texture.view,
			// 	depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Store }),
			// 	stencil_ops: None,
			// }),
			occlusion_query_set: None,
			timestamp_writes: None,
			multiview_mask: None,
		});

		render_pass.set_bind_group(0, camera_transform_bind_group, &[]);
		render_pass.set_bind_group(1, &gpu_bvh.bind_group, &[]);
		render_pass.set_bind_group(2, &tree_bind_group, &[]);
		render_pass.set_pipeline(&self.ray_marching_pipeline);
		render_pass.draw(0..3, 0..1);
	}
}
