use bevy::asset::{Handle, embedded_asset, load_embedded_asset};
use bevy::core_pipeline::tonemapping::tonemapping;
use bevy::core_pipeline::{Core3d, Core3dSystems};
use bevy::prelude::*;
use bevy::render::render_resource::{
	BindGroupLayoutDescriptor, CachedRenderPipelineId, FragmentState, PipelineCache,
	RenderPipelineDescriptor, ShaderType, UniformBuffer, VertexState,
};
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery, WgpuWrapper};
use bevy::render::view::ViewTarget;
use bevy::render::{Render, RenderApp, RenderSystems};

type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;

#[derive(Clone, Copy, Default, ShaderType)]
struct CrosshairUniform {
	screen_size: Vec4,
}

#[derive(Default)]
pub struct CrosshairPlugin;

impl Plugin for CrosshairPlugin {
	fn build(&self, app: &mut App) {
		embedded_asset!(app, "../shaders/crosshair.wgsl");
		let shader = load_embedded_asset!(app, "../shaders/crosshair.wgsl");
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app
			.insert_resource(CrosshairShader(shader))
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

#[derive(Resource)]
struct CrosshairShader(Handle<Shader>);

#[derive(Resource, Default)]
struct CrosshairResource {
	renderer: Option<CrosshairRenderer>,
	format: Option<wgpu::TextureFormat>,
}

impl CrosshairResource {
	fn ensure(
		&mut self,
		render_device: &RenderDevice,
		render_queue: &RenderQueue,
		pipeline_cache: &PipelineCache,
		shader: &Handle<Shader>,
		format: wgpu::TextureFormat,
	) {
		if self.format == Some(format) && self.renderer.is_some() { return; }
		self.renderer = Some(CrosshairRenderer::new(render_device, render_queue, pipeline_cache, shader, format));
		self.format = Some(format);
	}
}

struct CrosshairRenderer {
	pipeline: CachedRenderPipelineId,
	buffer: UniformBuffer<CrosshairUniform>,
	bind_group: GpuBindGroup,
}

impl CrosshairRenderer {
	fn new(
		render_device: &RenderDevice,
		render_queue: &RenderQueue,
		pipeline_cache: &PipelineCache,
		shader: &Handle<Shader>,
		format: wgpu::TextureFormat,
	) -> Self {
		let device = render_device.wgpu_device();
		let layout_descriptor = BindGroupLayoutDescriptor::new(
			"Crosshair BGL",
			&[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Uniform,
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
		);
		let layout = pipeline_cache.get_bind_group_layout(&layout_descriptor);
		let mut buffer = UniformBuffer::from(CrosshairUniform::default());
		buffer.set_label(Some("Crosshair Screen Size"));
		buffer.write_buffer(render_device, render_queue);
		let bind_group = WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			label: Some("Crosshair BG"),
			layout: &layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource: buffer.binding().expect("crosshair uniform buffer must be initialized"),
			}],
		}));
		let pipeline = pipeline_cache.queue_render_pipeline(RenderPipelineDescriptor {
			label: Some("Crosshair Pipeline".into()),
			layout: vec![layout_descriptor],
			vertex: VertexState {
				shader: shader.clone(),
				entry_point: Some("vs_main".into()),
				..default()
			},
			fragment: Some(FragmentState {
				shader: shader.clone(),
				entry_point: Some("fs_main".into()),
				targets: vec![Some(wgpu::ColorTargetState {
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
				..default()
			}),
			primitive: wgpu::PrimitiveState::default(),
			multisample: wgpu::MultisampleState::default(),
			..default()
		});

		Self { pipeline, buffer, bind_group }
	}
}

fn prepare_crosshair(
	render_device: Res<RenderDevice>,
	render_queue: Res<RenderQueue>,
	pipeline_cache: Res<PipelineCache>,
	shader: Res<CrosshairShader>,
	mut crosshair: ResMut<CrosshairResource>,
	views: Query<&ViewTarget>,
) {
	let Some(view_target) = views.iter().next() else { return };
	crosshair.ensure(
		&render_device,
		&render_queue,
		&pipeline_cache,
		&shader.0,
		view_target.main_texture_format(),
	);

	let Some(renderer) = crosshair.renderer.as_mut() else { return };
	let main_texture = view_target.main_texture();
	renderer.buffer.set(CrosshairUniform {
		screen_size: Vec4::new(main_texture.width() as f32, main_texture.height() as f32, 0.0, 0.0),
	});
	renderer.buffer.write_buffer(&render_device, &render_queue);
}

fn crosshair_pass(
	crosshair: Res<CrosshairResource>,
	pipeline_cache: Res<PipelineCache>,
	view: ViewQuery<&ViewTarget>,
	mut render_context: RenderContext,
) {
	let Some(renderer) = crosshair.renderer.as_ref() else { return };
	let Some(pipeline) = pipeline_cache.get_render_pipeline(renderer.pipeline) else { return };

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
	pass.set_pipeline(pipeline);
	pass.set_bind_group(0, &*renderer.bind_group, &[]);
	pass.draw(0..3, 0..1);
}
