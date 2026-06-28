use bevy::core_pipeline::tonemapping::tonemapping;
use bevy::core_pipeline::{Core3d, Core3dSystems};
use bevy::prelude::*;
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery, WgpuWrapper};
use bevy::render::view::ViewTarget;
use bevy::render::{Render, RenderApp, RenderSystems};

type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuRenderPipeline = WgpuWrapper<wgpu::RenderPipeline>;

#[derive(Default)]
pub struct CrosshairPlugin;

impl Plugin for CrosshairPlugin {
	fn build(&self, app: &mut App) {
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app
			.init_resource::<CrosshairResource>()
			.add_systems(Render, prepare_crosshair.in_set(RenderSystems::PrepareBindGroups))
			.add_systems(
				Core3d,
				crosshair_pass
					.in_set(Core3dSystems::PostProcess)
					.after(tonemapping),
			);
	}
}

#[derive(Resource, Default)]
struct CrosshairResource {
	renderer: Option<CrosshairRenderer>,
	format: Option<wgpu::TextureFormat>,
}

impl CrosshairResource {
	fn ensure(&mut self, device: &wgpu::Device, format: wgpu::TextureFormat) {
		if self.format == Some(format) && self.renderer.is_some() { return; }
		self.renderer = Some(CrosshairRenderer::new(device, format));
		self.format = Some(format);
	}
}

struct CrosshairRenderer {
	pipeline: GpuRenderPipeline,
	buffer: GpuBuffer,
	bind_group: GpuBindGroup,
}

impl CrosshairRenderer {
	fn new(device: &wgpu::Device, format: wgpu::TextureFormat) -> Self {
		let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("Crosshair Shader"),
			source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/crosshair.wgsl").into()),
		});
		let buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("Crosshair Screen Size"),
			size: 16,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		}));
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
		let bind_group = {
			let buffer = &*buffer;
			WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
				label: Some("Crosshair BG"),
				layout: &bgl,
				entries: &[wgpu::BindGroupEntry { binding: 0, resource: buffer.as_entire_binding() }],
			}))
		};
		let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: Some("Crosshair Pipeline Layout"),
			bind_group_layouts: &[Some(&bgl)],
			immediate_size: 0,
		});
		let pipeline = WgpuWrapper::new(device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
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
					format,
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
		}));

		Self { pipeline, buffer, bind_group }
	}
}

fn prepare_crosshair(
	render_device: Res<RenderDevice>,
	render_queue: Res<RenderQueue>,
	mut crosshair: ResMut<CrosshairResource>,
	views: Query<&ViewTarget>,
) {
	let Some(view_target) = views.iter().next() else { return };
	crosshair.ensure(render_device.wgpu_device(), view_target.main_texture_format());

	let Some(renderer) = crosshair.renderer.as_ref() else { return };
	let main_texture = view_target.main_texture();
	let mut bytes = [0u8; 8];
	bytes[0..4].copy_from_slice(&(main_texture.width() as f32).to_le_bytes());
	bytes[4..8].copy_from_slice(&(main_texture.height() as f32).to_le_bytes());
	render_queue.write_buffer(&renderer.buffer, 0, &bytes);
}

fn crosshair_pass(
	world: &World,
	view: ViewQuery<&ViewTarget>,
	mut render_context: RenderContext,
) {
	let crosshair = world.resource::<CrosshairResource>();
	let Some(renderer) = crosshair.renderer.as_ref() else { return };

	let encoder = render_context.command_encoder();
	let view = view.into_inner().main_texture_view();
	let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
		label: Some("Crosshair Pass"),
		color_attachments: &[Some(wgpu::RenderPassColorAttachment {
			view,
			resolve_target: None,
			depth_slice: None,
			ops: wgpu::Operations {
				load: wgpu::LoadOp::Load,
				store: wgpu::StoreOp::Store,
			},
		})],
		depth_stencil_attachment: None,
		occlusion_query_set: None,
		timestamp_writes: None,
		multiview_mask: None,
	});
	pass.set_pipeline(&renderer.pipeline);
	pass.set_bind_group(0, &*renderer.bind_group, &[]);
	pass.draw(0..3, 0..1);
}
