use std::sync::Arc;
use std::vec;

use glam::{Mat4};
use wgpu::util::DeviceExt;
use winit::{window::Window};

use crate::{debug_draw, gpu_objects::{matrix, mesh, texture}};

pub struct Renderer {
	pub surface: wgpu::Surface<'static>,
	pub device: wgpu::Device,
	pub queue: wgpu::Queue,
	pub config: wgpu::SurfaceConfiguration,
	pub is_surface_configured: bool,
	pub window: Arc<Window>,
	pub render_pipeline: wgpu::RenderPipeline,
	pub depth_texture: texture::Texture,
	pub camera_buffer: wgpu::Buffer,
	pub camera_bind_group: wgpu::BindGroup,
	pub crosshair_pipeline: wgpu::RenderPipeline,
	pub crosshair_buffer: wgpu::Buffer,
	pub crosshair_bind_group: wgpu::BindGroup,
	pub crosshair_format: wgpu::TextureFormat,
	pub debug_line_pipeline: wgpu::RenderPipeline,
	pub debug_tri_pipeline: wgpu::RenderPipeline,
}

impl Renderer {
	pub fn resize(&mut self, width: u32, height: u32) {
		if width > 0 && height > 0 {
			self.config.width = width;
			self.config.height = height;

			self.is_surface_configured = true;
			self.surface.configure(&self.device, &self.config);
			self.depth_texture = texture::Texture::create_depth_texture(&self.device, &self.config, "depth_texture");
		}
	}

	pub async fn new(window: Arc<Window>) -> anyhow::Result<Renderer> {
		let mut size = window.inner_size();
		size.width = size.width.max(1);
		size.height = size.height.max(1);
		// The instance is a handle to our GPU
		// BackendBit::PRIMARY => Vulkan + Metal + DX12 + Browser WebGPU
		let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
			#[cfg(not(target_arch = "wasm32"))]
			backends: wgpu::Backends::PRIMARY,
			#[cfg(target_arch = "wasm32")]
			backends: wgpu::Backends::GL,
			..Default::default()
		});

		let surface = instance.create_surface(window.clone()).unwrap();

		let adapter = instance.request_adapter(&wgpu::RequestAdapterOptions {
				power_preference: wgpu::PowerPreference::default(),
				compatible_surface: Some(&surface),
				force_fallback_adapter: false,
			})
			.await?;

		let (device, queue) = adapter.request_device(&wgpu::DeviceDescriptor {
				label: None,
				required_features: wgpu::Features::empty(),
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

		let crosshair_format = match surface_format {
			wgpu::TextureFormat::Rgba8UnormSrgb => wgpu::TextureFormat::Rgba8Unorm,
			wgpu::TextureFormat::Bgra8UnormSrgb => wgpu::TextureFormat::Bgra8Unorm,
			other => other,
		};

		let view_formats = if crosshair_format != surface_format { vec![crosshair_format] } else { vec![] };

		let config = wgpu::SurfaceConfiguration {
			usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
			format: surface_format,
			width: size.width,
			height: size.height,
			present_mode: surface_caps.present_modes[0],
			alpha_mode: surface_caps.alpha_modes[0],
			view_formats,
			desired_maximum_frame_latency: 2,
		};

		let depth_texture = texture::Texture::create_depth_texture(&device, &config, "depth_texture");

		let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("Shader"),
			source: wgpu::ShaderSource::Wgsl(include_str!("shaders/shader.wgsl").into()),
		});

		let camera_buffer = matrix::MatrixUniform::get_buffer(&device, 0);

		let render_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: Some("Render Pipeline Layout"),
			bind_group_layouts: &[&camera_buffer.2, &matrix::MatrixUniform::get_bind_group_layout(&device, 0)],
			push_constant_ranges: &[],
		});
		let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
			label: Some("Render Pipeline"),
			layout: Some(&render_pipeline_layout),

			vertex: wgpu::VertexState {
				module: &shader,
				entry_point: Some("vs_main"),
				buffers: &[<mesh::MeshVertex as mesh::Vertex>::desc()],
				compilation_options: wgpu::PipelineCompilationOptions::default(),
			},
			fragment: Some(wgpu::FragmentState {
				module: &shader,
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
			depth_stencil: Some(wgpu::DepthStencilState {
				format: texture::Texture::DEPTH_FORMAT,
				depth_write_enabled: true,
				depth_compare: wgpu::CompareFunction::Less,
				stencil: wgpu::StencilState::default(),
				bias: wgpu::DepthBiasState::default(),
			}),
			multisample: wgpu::MultisampleState {
				count: 1,
				mask: !0,
				alpha_to_coverage_enabled: false,
			},
			multiview: None,
			cache: None,
		});

		let crosshair_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("Crosshair Shader"),
			source: wgpu::ShaderSource::Wgsl(include_str!("shaders/crosshair.wgsl").into()),
		});
		let crosshair_buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("Crosshair Screen Size"),
			size: 8,
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
			bind_group_layouts: &[&crosshair_bgl],
			push_constant_ranges: &[],
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
			multiview: None,
			cache: None,
		});

		let debug_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("Debug Draw Shader"),
			source: wgpu::ShaderSource::Wgsl(include_str!("shaders/debug_render.wgsl").into()),
		});
		let debug_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: Some("Debug Pipeline Layout"),
			bind_group_layouts: &[&camera_buffer.2],
			push_constant_ranges: &[],
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
				multiview: None,
				cache: None,
			})
		};

		let debug_line_pipeline = make_debug_pipeline("Debug Line Pipeline", wgpu::PrimitiveTopology::LineList);
		let debug_tri_pipeline = make_debug_pipeline("Debug Tri Pipeline", wgpu::PrimitiveTopology::TriangleList);

		Ok(Self {
			surface,
			device,
			queue,
			config,
			is_surface_configured: false,
			window,
			render_pipeline,
			depth_texture,
			camera_buffer: camera_buffer.0,
			camera_bind_group: camera_buffer.1,
			crosshair_pipeline,
			crosshair_buffer,
			crosshair_bind_group,
			crosshair_format,
			debug_line_pipeline,
			debug_tri_pipeline,
		})
	}

	pub fn render(&mut self, camera_matrix: &Mat4, meshes: &Vec<(Arc<mesh::Mesh>, Mat4)>) -> Result<(), wgpu::SurfaceError> {
		self.window.request_redraw();

		if !self.is_surface_configured { return Ok(()); }

		self.queue.write_buffer(&self.camera_buffer, 0, bytemuck::cast_slice(&[matrix::MatrixUniform::from_mat4(camera_matrix)]));

		let output = self.surface.get_current_texture()?;

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
				depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
					view: &self.depth_texture.view,
					depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Store }),
					stencil_ops: None,
				}),
				occlusion_query_set: None,
				timestamp_writes: None,
			});

			render_pass.set_pipeline(&self.render_pipeline);

			for mesh in meshes.iter() {
				self.queue.write_buffer(&mesh.0.matrix_buffer, 0, bytemuck::cast_slice(&[matrix::MatrixUniform::from_mat4(&mesh.1)]));

				use mesh::DrawMesh;
				render_pass.draw_mesh(&mesh.0, &self.camera_bind_group);
			}
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
			});
			pass.set_pipeline(&self.crosshair_pipeline);
			pass.set_bind_group(0, &self.crosshair_bind_group, &[]);
			pass.draw(0..3, 0..1);
		}

		self.queue.submit(std::iter::once(encoder.finish()));
		output.present();

		Ok(())
	}
}
