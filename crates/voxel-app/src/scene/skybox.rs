use bevy::asset::{Handle, embedded_asset, load_embedded_asset};
use bevy::camera::{Camera, Projection};
use bevy::core_pipeline::{Core3d, Core3dSystems};
use bevy::prelude::*;
use bevy::render::render_resource::{
	BindGroupLayoutDescriptor, CachedRenderPipelineId, FragmentState, PipelineCache,
	RenderPipelineDescriptor, UniformBuffer, VertexState,
};
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery, WgpuWrapper};
use bevy::render::view::ViewTarget;
use bevy::render::{Extract, ExtractSchedule, Render, RenderApp, RenderSystems};
use bevy::transform::components::GlobalTransform;

use voxel_raster_renderer::render_node::voxel_raster_render_pass;
use voxel_ray_renderer::camera::CameraUniform;
use voxel_ray_renderer::render_node::voxel_render_pass;

type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;

#[derive(Default)]
pub struct SkyboxPlugin;

impl Plugin for SkyboxPlugin {
	fn build(&self, app: &mut App) {
		embedded_asset!(app, "../shaders/skybox.wgsl");
		let shader = load_embedded_asset!(app, "../shaders/skybox.wgsl");
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app
			.insert_resource(SkyboxShader(shader))
			.init_resource::<SkyboxCamera>()
			.init_resource::<SkyboxResource>()
			.add_systems(ExtractSchedule, extract_skybox_camera)
			.add_systems(Render, prepare_skybox.in_set(RenderSystems::PrepareBindGroups))
			.add_systems(
				Core3d,
				skybox_pass
					.in_set(Core3dSystems::MainPass)
					.before(voxel_render_pass)
					.before(voxel_raster_render_pass),
			);
	}
}

#[derive(Resource)]
struct SkyboxShader(Handle<Shader>);

#[derive(Resource, Default)]
struct SkyboxCamera(Option<(Transform, Projection)>);

#[derive(Resource, Default)]
struct SkyboxResource {
	renderer: Option<SkyboxRenderer>,
	format: Option<wgpu::TextureFormat>,
}

impl SkyboxResource {
	fn ensure(
		&mut self,
		render_device: &RenderDevice,
		render_queue: &RenderQueue,
		pipeline_cache: &PipelineCache,
		shader: &Handle<Shader>,
		format: wgpu::TextureFormat,
	) {
		if self.format == Some(format) && self.renderer.is_some() { return; }
		self.renderer = Some(SkyboxRenderer::new(render_device, render_queue, pipeline_cache, shader, format));
		self.format = Some(format);
	}
}

struct SkyboxRenderer {
	pipeline: CachedRenderPipelineId,
	buffer: UniformBuffer<CameraUniform>,
	bind_group: GpuBindGroup,
}

impl SkyboxRenderer {
	fn new(
		render_device: &RenderDevice,
		render_queue: &RenderQueue,
		pipeline_cache: &PipelineCache,
		shader: &Handle<Shader>,
		format: wgpu::TextureFormat,
	) -> Self {
		let device = render_device.wgpu_device();
		let layout_descriptor = BindGroupLayoutDescriptor::new(
			"Skybox Camera Bind Group Layout",
			&[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Uniform,
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
		);
		let layout = pipeline_cache.get_bind_group_layout(&layout_descriptor);
		let mut buffer = UniformBuffer::from(CameraUniform::default());
		buffer.set_label(Some("camera_buffer"));
		buffer.write_buffer(render_device, render_queue);
		let bind_group = WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource: buffer.binding().expect("skybox camera uniform buffer must be initialized"),
			}],
			label: Some("skybox_camera_bind_group"),
		}));
		let pipeline = pipeline_cache.queue_render_pipeline(RenderPipelineDescriptor {
			label: Some("Skybox Pipeline".into()),
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
					blend: Some(wgpu::BlendState::REPLACE),
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

fn extract_skybox_camera(
	mut skybox_camera: ResMut<SkyboxCamera>,
	cameras: Extract<Query<(&Camera, &Projection, &GlobalTransform)>>,
) {
	skybox_camera.0 = cameras
		.iter()
		.find(|(camera, _, _)| camera.is_active)
		.map(|(_, projection, global_transform)| (global_transform.compute_transform(), projection.clone()));
}

fn prepare_skybox(
	render_device: Res<RenderDevice>,
	render_queue: Res<RenderQueue>,
	pipeline_cache: Res<PipelineCache>,
	shader: Res<SkyboxShader>,
	mut skybox: ResMut<SkyboxResource>,
	skybox_camera: Res<SkyboxCamera>,
	views: Query<&ViewTarget>,
) {
	let Some(view_target) = views.iter().next() else { return };
	skybox.ensure(
		&render_device,
		&render_queue,
		&pipeline_cache,
		&shader.0,
		view_target.main_texture_format(),
	);

	let Some(renderer) = skybox.renderer.as_mut() else { return };
	let Some((transform, projection)) = skybox_camera.0.as_ref() else { return };
	let Ok(uniform) = CameraUniform::from_camera(transform, projection) else { return };
	renderer.buffer.set(uniform);
	renderer.buffer.write_buffer(&render_device, &render_queue);
}

fn skybox_pass(
	skybox: Res<SkyboxResource>,
	pipeline_cache: Res<PipelineCache>,
	view: ViewQuery<&ViewTarget>,
	mut render_context: RenderContext,
) {
	let Some(renderer) = skybox.renderer.as_ref() else { return };
	let Some(pipeline) = pipeline_cache.get_render_pipeline(renderer.pipeline) else { return };

	let view_target = view.into_inner();
	let color_attachment = view_target.get_color_attachment();
	let encoder = render_context.command_encoder();
	let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
		label: Some("Skybox Pass"),
		color_attachments: &[Some(color_attachment)],
		depth_stencil_attachment: None,
		occlusion_query_set: None,
		timestamp_writes: None,
		multiview_mask: None,
	});
	pass.set_pipeline(pipeline);
	pass.set_bind_group(0, &*renderer.bind_group, &[]);
	pass.draw(0..3, 0..1);
}
