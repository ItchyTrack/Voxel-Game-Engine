pub struct CrosshairRenderer {
	pub crosshair_pipeline: wgpu::RenderPipeline,
	pub crosshair_buffer: wgpu::Buffer,
	pub crosshair_bind_group: wgpu::BindGroup,
	pub crosshair_format: wgpu::TextureFormat,
}

impl CrosshairRenderer {
	pub fn new(device: &wgpu::Device, config: &wgpu::SurfaceConfiguration) -> anyhow::Result<Self> {
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
					format: config.format,
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

		Ok(Self {
			crosshair_pipeline: crosshair_pipeline,
			crosshair_buffer: crosshair_buffer,
			crosshair_bind_group: crosshair_bind_group,
			crosshair_format: config.format,
		})
	}

	pub fn render(&self, config: &wgpu::SurfaceConfiguration, encoder: &mut wgpu::CommandEncoder, queue: &wgpu::Queue, output: &wgpu::SurfaceTexture) {
		queue.write_buffer(&self.crosshair_buffer, 0, bytemuck::cast_slice(&[config.width as f32, config.height as f32]));
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
}
