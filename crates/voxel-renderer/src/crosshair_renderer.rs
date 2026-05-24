use bevy::ecs::resource::Resource;

pub struct CrosshairRenderer {
	pub pipeline: wgpu::RenderPipeline,
	pub buffer: wgpu::Buffer,
	pub bind_group: wgpu::BindGroup,
}

impl CrosshairRenderer {
	pub fn new(device: &wgpu::Device, color_format: wgpu::TextureFormat) -> Self {
		let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("Crosshair Shader"),
			source: wgpu::ShaderSource::Wgsl(include_str!("shaders/crosshair.wgsl").into()),
		});
		let buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("Crosshair Screen Size"),
			size: 16,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		});
		let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: Some("Crosshair BGL"),
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Uniform,
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
		});
		let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			label: Some("Crosshair BG"),
			layout: &bgl,
			entries: &[wgpu::BindGroupEntry { binding: 0, resource: buffer.as_entire_binding() }],
		});
		let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: Some("Crosshair Pipeline Layout"),
			bind_group_layouts: &[&bgl],
			push_constant_ranges: &[],
		});
		let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
			label: Some("Crosshair Pipeline"),
			layout: Some(&pipeline_layout),
			vertex: wgpu::VertexState {
				module: &shader,
				entry_point: Some("vs_main"),
				buffers: &[],
				compilation_options: wgpu::PipelineCompilationOptions::default(),
			},
			fragment: Some(wgpu::FragmentState {
				module: &shader,
				entry_point: Some("fs_main"),
				targets: &[Some(wgpu::ColorTargetState {
					format: color_format,
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

		Self { pipeline, buffer, bind_group }
	}
}

#[derive(Resource, Default)]
pub struct CrosshairResource {
	pub renderer: Option<CrosshairRenderer>,
	pub format: Option<wgpu::TextureFormat>,
}

impl CrosshairResource {
	pub fn ensure(&mut self, device: &wgpu::Device, format: wgpu::TextureFormat) {
		if self.format == Some(format) && self.renderer.is_some() { return; }
		self.renderer = Some(CrosshairRenderer::new(device, format));
		self.format = Some(format);
	}
}
