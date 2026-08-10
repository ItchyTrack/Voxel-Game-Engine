use bevy::asset::{Handle, embedded_asset, load_embedded_asset};
use bevy::core_pipeline::core_3d::{main_opaque_pass_3d, main_transparent_pass_3d};
use bevy::core_pipeline::{Core3d, Core3dSystems};
use bevy::prelude::*;
use bevy::render::render_resource::{
	BindGroup, BindGroupEntries, BindGroupLayoutDescriptor, BindGroupLayoutEntries, CachedRenderPipelineId,
	FragmentState, PipelineCache, RenderPipelineDescriptor, ShaderStages, StoreOp, VertexState,
	binding_types::uniform_buffer,
};
use bevy::render::renderer::{RenderContext, RenderDevice, ViewQuery};
use bevy::render::view::{ViewDepthTexture, ViewTarget, ViewUniform, ViewUniformOffset, ViewUniforms};
use bevy::render::{Render, RenderApp, RenderSystems};

use voxel_ray_renderer::render_node::voxel_render_pass;

#[derive(Default)]
pub struct SkyboxPlugin;

impl Plugin for SkyboxPlugin {
	fn build(&self, app: &mut App) {
		embedded_asset!(app, "../shaders/skybox.wgsl");
		let shader = load_embedded_asset!(app, "../shaders/skybox.wgsl");
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app
			.insert_resource(SkyboxShader(shader))
			.init_resource::<SkyboxResource>()
			.add_systems(Render, prepare_skybox.in_set(RenderSystems::PrepareBindGroups))
			.add_systems(
				Core3d,
				skybox_pass
					.in_set(Core3dSystems::MainPass)
					.after(main_opaque_pass_3d)
					.before(main_transparent_pass_3d)
					.before(voxel_render_pass),
			);
	}
}

#[derive(Resource)]
struct SkyboxShader(Handle<Shader>);

#[derive(Resource, Default)]
struct SkyboxResource {
	renderer: Option<SkyboxRenderer>,
	format: Option<wgpu::TextureFormat>,
}

impl SkyboxResource {
	fn ensure(
		&mut self,
		pipeline_cache: &PipelineCache,
		shader: &Handle<Shader>,
		format: wgpu::TextureFormat,
	) {
		if self.format == Some(format) && self.renderer.is_some() { return; }
		self.renderer = Some(SkyboxRenderer::new(pipeline_cache, shader, format));
		self.format = Some(format);
	}
}

struct SkyboxRenderer {
	pipeline: CachedRenderPipelineId,
	layout: BindGroupLayoutDescriptor,
	bind_group: Option<BindGroup>,
}

impl SkyboxRenderer {
	fn new(pipeline_cache: &PipelineCache, shader: &Handle<Shader>, format: wgpu::TextureFormat) -> Self {
		let layout = BindGroupLayoutDescriptor::new(
			"Skybox View Bind Group Layout",
			&BindGroupLayoutEntries::single(ShaderStages::FRAGMENT, uniform_buffer::<ViewUniform>(true)),
		);
		let pipeline = pipeline_cache.queue_render_pipeline(RenderPipelineDescriptor {
			label: Some("Skybox Pipeline".into()),
			layout: vec![layout.clone()],
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
					blend: Some(wgpu::BlendState::REPLACE),
					write_mask: wgpu::ColorWrites::ALL,
				})],
				..default()
			}),
			primitive: wgpu::PrimitiveState::default(),
			depth_stencil: Some(wgpu::DepthStencilState {
				format: wgpu::TextureFormat::Depth32Float,
				depth_write_enabled: Some(false),
				depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
				stencil: wgpu::StencilState::default(),
				bias: wgpu::DepthBiasState::default(),
			}),
			multisample: wgpu::MultisampleState::default(),
			..default()
		});
		Self { pipeline, layout, bind_group: None }
	}
}

fn prepare_skybox(
	render_device: Res<RenderDevice>,
	pipeline_cache: Res<PipelineCache>,
	view_uniforms: Res<ViewUniforms>,
	shader: Res<SkyboxShader>,
	mut skybox: ResMut<SkyboxResource>,
	views: Query<&ViewTarget, With<ViewUniformOffset>>,
) {
	let Some(view_target) = views.iter().next() else { return };
	skybox.ensure(&pipeline_cache, &shader.0, view_target.main_texture_format());
	let Some(renderer) = skybox.renderer.as_mut() else { return };
	let Some(view_binding) = view_uniforms.uniforms.binding() else {
		renderer.bind_group = None;
		return;
	};
	renderer.bind_group = Some(render_device.create_bind_group(
		"skybox_view_bind_group",
		&pipeline_cache.get_bind_group_layout(&renderer.layout),
		&BindGroupEntries::single(view_binding),
	));
}

fn skybox_pass(
	skybox: Res<SkyboxResource>,
	pipeline_cache: Res<PipelineCache>,
	view: ViewQuery<(&ViewTarget, &ViewDepthTexture, &ViewUniformOffset)>,
	mut render_context: RenderContext,
) {
	let Some(renderer) = skybox.renderer.as_ref() else { return };
	let Some(pipeline) = pipeline_cache.get_render_pipeline(renderer.pipeline) else { return };
	let Some(bind_group) = renderer.bind_group.as_ref() else { return };

	let (view_target, view_depth, view_offset) = view.into_inner();
	let color_attachment = view_target.get_color_attachment();
	let encoder = render_context.command_encoder();
	let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
		label: Some("Skybox Pass"),
		color_attachments: &[Some(color_attachment)],
		depth_stencil_attachment: Some(view_depth.get_attachment(StoreOp::Store)),
		occlusion_query_set: None,
		timestamp_writes: None,
		multiview_mask: None,
	});
	pass.set_pipeline(pipeline);
	pass.set_bind_group(0, bind_group, &[view_offset.offset]);
	pass.draw(0..3, 0..1);
}
