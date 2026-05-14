use crate::debug_draw;
use wgpu::util::DeviceExt;
pub struct DebugDrawRenderer {
	pub debug_line_pipeline: wgpu::RenderPipeline,
	pub debug_tri_pipeline: wgpu::RenderPipeline,
}

impl DebugDrawRenderer {
	pub fn new(device: &wgpu::Device, config: &wgpu::SurfaceConfiguration, camera_bind_group_layout: &wgpu::BindGroupLayout) -> anyhow::Result<Self> {
		let debug_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("Debug Draw Shader"),
			source: wgpu::ShaderSource::Wgsl(include_str!("shaders/debug_render.wgsl").into()),
		});
		let debug_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: Some("Debug Pipeline Layout"),
			bind_group_layouts: &[Some(&camera_bind_group_layout)],
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
				depth_stencil: None, // overlay - no depth test
				multisample: wgpu::MultisampleState::default(),
				multiview_mask: None,
				cache: None,
			})
		};

		let debug_line_pipeline = make_debug_pipeline("Debug Line Pipeline", wgpu::PrimitiveTopology::LineList);
		let debug_tri_pipeline = make_debug_pipeline("Debug Tri Pipeline", wgpu::PrimitiveTopology::TriangleList);

		return Ok(Self {
			debug_line_pipeline,
			debug_tri_pipeline,
		})
	}

	pub fn render(&self, device: &wgpu::Device, encoder: &mut wgpu::CommandEncoder, camera_bind_group: &wgpu::BindGroup, view: &wgpu::TextureView,) {
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

			if has_lines && (bytemuck::cast_slice::<_, u8>(&batch.lines).len() as u64) < (device.limits().max_buffer_size) {
				let line_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
					label: Some("Debug Line VB"),
					contents: bytemuck::cast_slice(&batch.lines),
					usage: wgpu::BufferUsages::VERTEX,
				});
				pass.set_pipeline(&self.debug_line_pipeline);
				pass.set_bind_group(0, camera_bind_group, &[]);
				pass.set_vertex_buffer(0, line_buf.slice(..));
				pass.draw(0..batch.lines.len() as u32, 0..1);
			}

			if has_tris && (bytemuck::cast_slice::<_, u8>(&batch.triangles).len() as u64) < (device.limits().max_buffer_size) {
				let tri_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
					label: Some("Debug Tri VB"),
					contents: bytemuck::cast_slice(&batch.triangles),
					usage: wgpu::BufferUsages::VERTEX,
				});
				pass.set_pipeline(&self.debug_tri_pipeline);
				pass.set_bind_group(0, camera_bind_group, &[]);
				pass.set_vertex_buffer(0, tri_buf.slice(..));
				pass.draw(0..batch.triangles.len() as u32, 0..1);
			}
		}
	}
}
