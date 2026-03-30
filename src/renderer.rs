use std::collections::HashMap;
use std::sync::Arc;
use std::vec;

use glam::IVec3;
use tracy_client::span;
use wgpu::util::DeviceExt;
use winit::{window::Window};

use crate::gpu_objects::gpu_bvh;
use crate::gpu_objects::gpu_grid_tree;
use crate::gpu_objects::packed_dynamic_buffer::PackedDynamicBuffer;
use crate::physics::bvh;
use crate::player::camera;
use crate::pose::Pose;
use crate::{debug_draw, gpu_objects::matrix};

pub struct Renderer {
	pub surface: wgpu::Surface<'static>,
	pub device: wgpu::Device,
	pub queue: wgpu::Queue,
	pub config: wgpu::SurfaceConfiguration,
	pub is_surface_configured: bool,
	pub packed_64_tree_dynamic_buffer: PackedDynamicBuffer,
	pub window: Arc<Window>,
	// pub render_pipeline: wgpu::RenderPipeline,
	// pub depth_texture: texture::Texture,
	pub ray_marching_pipeline: wgpu::RenderPipeline,
	pub camera_buffer: wgpu::Buffer,
	pub camera_bind_group: wgpu::BindGroup,
	pub camera_transform_buffer: wgpu::Buffer,
	pub camera_transform_bind_group: wgpu::BindGroup,
	pub crosshair_pipeline: wgpu::RenderPipeline,
	pub crosshair_buffer: wgpu::Buffer,
	pub crosshair_bind_group: wgpu::BindGroup,
	pub crosshair_format: wgpu::TextureFormat,
	pub debug_line_pipeline: wgpu::RenderPipeline,
	pub debug_tri_pipeline: wgpu::RenderPipeline,
}

#[cfg(target_arch = "wasm32")]
const MAX_SCREEN_SIZE: u32 = 2048;

impl Renderer {
	pub fn resize(&mut self, width: u32, height: u32) {
		if width > 0 && height > 0 {
			#[cfg(target_arch = "wasm32")]
			if width < height {
				self.config.width = (height.min(MAX_SCREEN_SIZE) * width) / height;
				self.config.height = height.min(MAX_SCREEN_SIZE);
			} else {
				self.config.width = width.min(MAX_SCREEN_SIZE);
				self.config.height = (width.min(MAX_SCREEN_SIZE) * height) / width;
			}
			#[cfg(not(target_arch = "wasm32"))]
			{
				self.config.width = width;
				self.config.height = height;
			}

			self.is_surface_configured = true;
			self.surface.configure(&self.device, &self.config);
			// self.depth_texture = texture::Texture::create_depth_texture(&self.device, &self.config, "depth_texture");
		}
	}

	pub async fn new(window: Arc<Window>) -> anyhow::Result<Renderer> {
		let mut size = window.inner_size();
		size.width = size.width.max(1);
		size.height = size.height.max(1);
		#[cfg(target_arch = "wasm32")]
		if size.width < size.height {
			size.width = (size.height.min(MAX_SCREEN_SIZE) * size.width) / size.height;
			size.height = size.height.min(MAX_SCREEN_SIZE);
		} else {
			size.width = size.width.min(MAX_SCREEN_SIZE);
			size.height = (size.width.min(MAX_SCREEN_SIZE) * size.height) / size.width;
		}

		// #[cfg(target_arch = "wasm32")]
		// if !wgpu::util::is_browser_webgpu_supported().await {
		// 	panic!("Browser does not support webgpu")
		// }

		// The instance is a handle to our GPU
		// BackendBit::PRIMARY => Vulkan + Metal + DX12 + Browser WebGPU
		let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
			#[cfg(not(target_arch = "wasm32"))]
			backends: wgpu::Backends::PRIMARY,
			#[cfg(target_arch = "wasm32")]
			backends: wgpu::Backends::BROWSER_WEBGPU,
			..wgpu::InstanceDescriptor::new_without_display_handle()
		});

		let surface = instance.create_surface(window.clone()).unwrap();
		// let surface = {
		// 	#[cfg(target_arch = "wasm32")]
		// 	{
		// 		use wasm_bindgen::JsCast;
		// 		use web_sys::HtmlCanvasElement;

		// 		let canvas = web_sys::window()
		// 			.unwrap()
		// 			.document()
		// 			.unwrap()
		// 			.get_element_by_id("your-canvas-id")  // match your HTML
		// 			.unwrap()
		// 			.dyn_into::<HtmlCanvasElement>()
		// 			.unwrap();

		// 		instance
		// 			.create_surface(wgpu::SurfaceTarget::Canvas(canvas))
		// 			.unwrap()
		// 	}
		// 	#[cfg(not(target_arch = "wasm32"))]
		// 	{
		// 		instance.create_surface(window.clone()).unwrap()
		// 	}
		// };

		let adapter = instance.request_adapter(&wgpu::RequestAdapterOptions {
				power_preference: wgpu::PowerPreference::default(),
				compatible_surface: Some(&surface),
				force_fallback_adapter: false,
			})
			.await?;

		let (device, queue) = adapter.request_device(&wgpu::DeviceDescriptor {
				label: None,
				required_features: wgpu::Features::default(),
				experimental_features: wgpu::ExperimentalFeatures::disabled(),
				// WebGL doesn't support all of wgpu's features, so if
				// we're building for the web we'll have to disable some.
				required_limits: if cfg!(target_arch = "wasm32") { wgpu::Limits::downlevel_webgl2_defaults() } else { wgpu::Limits::default() },
				memory_hints: Default::default(),
				trace: wgpu::Trace::Off,
			})
			.await?;

		let surface_caps = surface.get_capabilities(&adapter);

		let surface_format = surface_caps.formats.iter().find(|f| f.is_srgb()).copied().unwrap_or(surface_caps.formats[0]);

		let crosshair_format = surface_format;

		let config = wgpu::SurfaceConfiguration {
			usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
			format: surface_format,
			width: size.width,
			height: size.height,
			present_mode: surface_caps.present_modes[0],
			alpha_mode: surface_caps.alpha_modes[0],
			view_formats: vec![surface_format],
			desired_maximum_frame_latency: 2,
		};

		// let depth_texture = texture::Texture::create_depth_texture(&device, &config, "depth_texture");

		let camera_buffer = matrix::MatrixUniform::get_buffer(&device, 0);
		let camera_transform_buffer = camera::CameraUniform::get_buffer(&device, 0);

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
			}],
			label: Some("bvh_bind_group_layout"),
		});
		let shader_src = concat!(
			include_str!("shaders/bvh_raycast.wgsl"),
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
				Some(&camera_transform_buffer.2),
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

		let crosshair_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("Crosshair Shader"),
			source: wgpu::ShaderSource::Wgsl(include_str!("shaders/crosshair.wgsl").into()),
		});
		let crosshair_buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("Crosshair Screen Size"),
			size: 16,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		});
		let crosshair_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: Some("Crosshair BGL"),
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
				count: None,
			}],
		});
		let crosshair_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			label: Some("Crosshair BG"),
			layout: &crosshair_bgl,
			entries: &[wgpu::BindGroupEntry { binding: 0, resource: crosshair_buffer.as_entire_binding() }],
		});
		let crosshair_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: Some("Crosshair Pipeline Layout"),
			bind_group_layouts: &[Some(&crosshair_bgl)],
			immediate_size: 0,
		});
		let crosshair_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
			label: Some("Crosshair Pipeline"),
			layout: Some(&crosshair_pipeline_layout),
			vertex: wgpu::VertexState {
				module: &crosshair_shader,
				entry_point: Some("vs_main"),
				buffers: &[],
				compilation_options: wgpu::PipelineCompilationOptions::default(),
			},
			fragment: Some(wgpu::FragmentState {
				module: &crosshair_shader,
				entry_point: Some("fs_main"),
				targets: &[Some(wgpu::ColorTargetState {
					format: crosshair_format,
					blend: Some(wgpu::BlendState {
						color: wgpu::BlendComponent {
							src_factor: wgpu::BlendFactor::OneMinusDst,
							dst_factor: wgpu::BlendFactor::Zero,
							operation: wgpu::BlendOperation::Add,
						},
						alpha: wgpu::BlendComponent::OVER,
					}),
					write_mask: wgpu::ColorWrites::ALL,
				})],
				compilation_options: wgpu::PipelineCompilationOptions::default(),
			}),
			primitive: wgpu::PrimitiveState::default(),
			depth_stencil: None,
			multisample: wgpu::MultisampleState::default(),
			multiview_mask: None,
			cache: None,
		});

		let debug_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("Debug Draw Shader"),
			source: wgpu::ShaderSource::Wgsl(include_str!("shaders/debug_render.wgsl").into()),
		});
		let debug_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: Some("Debug Pipeline Layout"),
			bind_group_layouts: &[Some(&camera_buffer.2)],
			immediate_size: 0,
		});

		let make_debug_pipeline = |label: &str, topology: wgpu::PrimitiveTopology| -> wgpu::RenderPipeline {
			device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
				label: Some(label),
				layout: Some(&debug_pipeline_layout),
				vertex: wgpu::VertexState {
					module: &debug_shader,
					entry_point: Some("vs_main"),
					buffers: &[debug_draw::DebugVertex::desc()],
					compilation_options: wgpu::PipelineCompilationOptions::default(),
				},
				fragment: Some(wgpu::FragmentState {
					module: &debug_shader,
					entry_point: Some("fs_main"),
					targets: &[Some(wgpu::ColorTargetState {
						format: config.format,
						blend: Some(wgpu::BlendState::ALPHA_BLENDING),
						write_mask: wgpu::ColorWrites::ALL,
					})],
					compilation_options: wgpu::PipelineCompilationOptions::default(),
				}),
				primitive: wgpu::PrimitiveState {
					topology,
					strip_index_format: None,
					front_face: wgpu::FrontFace::Ccw,
					cull_mode: None,
					polygon_mode: wgpu::PolygonMode::Fill,
					unclipped_depth: false,
					conservative: false,
				},
				depth_stencil: None, // overlay — no depth test
				multisample: wgpu::MultisampleState::default(),
				multiview_mask: None,
				cache: None,
			})
		};

		let debug_line_pipeline = make_debug_pipeline("Debug Line Pipeline", wgpu::PrimitiveTopology::LineList);
		let debug_tri_pipeline = make_debug_pipeline("Debug Tri Pipeline", wgpu::PrimitiveTopology::TriangleList);

		let packed_64_tree_dynamic_buffer = PackedDynamicBuffer::new(&device, size_of::<gpu_grid_tree::GpuGridTreeNode>() as u32, wgpu::BufferUsages::STORAGE);
		if let Err(err) = packed_64_tree_dynamic_buffer {
			println!("{}", err);
			return Err(anyhow::Error::msg(err));
		}

		Ok(Self {
			surface,
			device,
			queue,
			config,
			is_surface_configured: false,
			window,
			// render_pipeline,
			// depth_texture,
			ray_marching_pipeline,
			camera_buffer: camera_buffer.0,
			camera_bind_group: camera_buffer.1,
			camera_transform_buffer: camera_transform_buffer.0,
			camera_transform_bind_group: camera_transform_buffer.1,
			crosshair_pipeline,
			crosshair_buffer,
			crosshair_bind_group,
			crosshair_format,
			debug_line_pipeline,
			debug_tri_pipeline,
			packed_64_tree_dynamic_buffer: packed_64_tree_dynamic_buffer.unwrap(),
		})
	}

	pub fn render(&mut self, camera: &camera::Camera, bvh: &bvh::BVH<(u32, u32, IVec3)>, gpu_grid_tree_id_to_id_poses: &HashMap<(u32, u32, IVec3), (u32, Pose)>) -> Result<(), wgpu::CurrentSurfaceTexture> {
		self.window.request_redraw();

		let tree_bind_group_layout = self.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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
			label: Some("tree_bind_group_layout"),
		});
		let tree_bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &tree_bind_group_layout,
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

		if !self.is_surface_configured { return Ok(()); }
		self.queue.write_buffer(&self.camera_buffer, 0, bytemuck::cast_slice(&[matrix::MatrixUniform::from_mat4(&camera.build_view_projection_matrix())]));
		self.queue.write_buffer(&self.camera_transform_buffer, 0, bytemuck::cast_slice(&[camera::CameraUniform::from_camera(camera)]));

		let output = {
			let _zone = span!("Finish Last Render");
			match self.surface.get_current_texture() {
				wgpu::CurrentSurfaceTexture::Success(surface_texture) => surface_texture,
				wgpu::CurrentSurfaceTexture::Suboptimal(surface_texture) => return Err(wgpu::CurrentSurfaceTexture::Suboptimal(surface_texture)),
				wgpu::CurrentSurfaceTexture::Timeout => 					return Err(wgpu::CurrentSurfaceTexture::Timeout),
				wgpu::CurrentSurfaceTexture::Occluded => 					return Err(wgpu::CurrentSurfaceTexture::Occluded),
				wgpu::CurrentSurfaceTexture::Outdated => 					return Err(wgpu::CurrentSurfaceTexture::Outdated),
				wgpu::CurrentSurfaceTexture::Lost => 						return Err(wgpu::CurrentSurfaceTexture::Lost),
				wgpu::CurrentSurfaceTexture::Validation => 					return Err(wgpu::CurrentSurfaceTexture::Validation),
			}
		};

		let _zone = span!("Render");

		let view = output.texture.create_view(&wgpu::TextureViewDescriptor::default());

		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Render Encoder") });
		{
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

			// render_pass.set_pipeline(&self.render_pipeline);

			// self.packed_mesh_buffer.render(&self.device, &self.queue, &mut render_pass, &self.camera_bind_group, &meshes);
			render_pass.set_bind_group(0, &self.camera_transform_bind_group, &[]);
			let gpu_bvh = gpu_bvh::GpuBvh::from_bvh(&self.device, bvh, gpu_grid_tree_id_to_id_poses);
			render_pass.set_bind_group(1, &gpu_bvh.bind_group, &[]);
			render_pass.set_bind_group(2, &tree_bind_group, &[]);
			render_pass.set_pipeline(&self.ray_marching_pipeline);
			render_pass.draw(0..3, 0..1);

			// for mesh in meshes.iter() {
			// 	self.queue.write_buffer(&mesh.0.matrix_buffer, 0, bytemuck::cast_slice(&[matrix::MatrixUniform::from_mat4(&mesh.1)]));

			// 	use mesh::DrawMesh;
			// 	render_pass.draw_mesh(&mesh.0, &self.camera_bind_group);
			// }
		}

		{
			let batch = debug_draw::take_batch();
			let has_lines = !batch.lines.is_empty();
			let has_tris = !batch.triangles.is_empty();

			if has_lines || has_tris {
				let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
					label: Some("Debug Draw Pass"),
					color_attachments: &[Some(wgpu::RenderPassColorAttachment {
						view: &view,
						resolve_target: None,
						depth_slice: None,
						ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
					})],
					depth_stencil_attachment: None,
					occlusion_query_set: None,
					timestamp_writes: None,
					multiview_mask: None,
				});

				if has_lines {
					let line_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
						label: Some("Debug Line VB"),
						contents: bytemuck::cast_slice(&batch.lines),
						usage: wgpu::BufferUsages::VERTEX,
					});
					pass.set_pipeline(&self.debug_line_pipeline);
					pass.set_bind_group(0, &self.camera_bind_group, &[]);
					pass.set_vertex_buffer(0, line_buf.slice(..));
					pass.draw(0..batch.lines.len() as u32, 0..1);
				}

				if has_tris {
					let tri_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
						label: Some("Debug Tri VB"),
						contents: bytemuck::cast_slice(&batch.triangles),
						usage: wgpu::BufferUsages::VERTEX,
					});
					pass.set_pipeline(&self.debug_tri_pipeline);
					pass.set_bind_group(0, &self.camera_bind_group, &[]);
					pass.set_vertex_buffer(0, tri_buf.slice(..));
					pass.draw(0..batch.triangles.len() as u32, 0..1);
				}
			}
		}

		{
			self.queue.write_buffer(&self.crosshair_buffer, 0, bytemuck::cast_slice(&[self.config.width as f32, self.config.height as f32]));
			let crosshair_view = output.texture.create_view(&wgpu::TextureViewDescriptor {
				format: Some(self.crosshair_format),
				..Default::default()
			});
			let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
				label: Some("Crosshair Pass"),
				color_attachments: &[Some(wgpu::RenderPassColorAttachment {
					view: &crosshair_view,
					resolve_target: None,
					depth_slice: None,
					ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
				})],
				depth_stencil_attachment: None,
				occlusion_query_set: None,
				timestamp_writes: None,
				multiview_mask: None,
			});
			pass.set_pipeline(&self.crosshair_pipeline);
			pass.set_bind_group(0, &self.crosshair_bind_group, &[]);
			pass.draw(0..3, 0..1);
		}
		{
			let _zone = span!("Submit");
			self.queue.submit(std::iter::once(encoder.finish()));
		}
		{
			let _zone = span!("Present");
			output.present();
		}

		Ok(())
	}
}
