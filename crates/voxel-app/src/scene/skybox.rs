use bevy::camera::{Camera, Projection};
use bevy::core_pipeline::{Core3d, Core3dSystems};
use bevy::prelude::*;
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery};
use bevy::render::view::ViewTarget;
use bevy::render::{Extract, ExtractSchedule, Render, RenderApp, RenderSystems};
use bevy::transform::components::GlobalTransform;

use voxel_renderer::camera::CameraUniform;
use voxel_renderer::render_node::voxel_render_pass;
use bevy::render::renderer::WgpuWrapper;

type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuRenderPipeline = WgpuWrapper<wgpu::RenderPipeline>;

#[derive(Default)]
pub struct SkyboxPlugin;

impl Plugin for SkyboxPlugin {
	fn build(&self, app: &mut App) {
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app
			.init_resource::<SkyboxCamera>()
			.init_resource::<SkyboxResource>()
			.add_systems(ExtractSchedule, extract_skybox_camera)
			.add_systems(Render, prepare_skybox.in_set(RenderSystems::PrepareBindGroups))
			.add_systems(
				Core3d,
				skybox_pass
					.in_set(Core3dSystems::MainPass)
					.before(voxel_render_pass),
			);
	}
}

#[derive(Resource, Default)]
struct SkyboxCamera(Option<(Transform, Projection)>);

#[derive(Resource, Default)]
struct SkyboxResource {
	renderer: Option<SkyboxRenderer>,
	format: Option<wgpu::TextureFormat>,
}

impl SkyboxResource {
	fn ensure(&mut self, device: &wgpu::Device, format: wgpu::TextureFormat) {
		if self.format == Some(format) && self.renderer.is_some() { return; }
		self.renderer = Some(SkyboxRenderer::new(device, format));
		self.format = Some(format);
	}
}

struct SkyboxRenderer {
	pipeline: GpuRenderPipeline,
	buffer: GpuBuffer,
	bind_group: GpuBindGroup,
}

impl SkyboxRenderer {
	fn new(device: &wgpu::Device, format: wgpu::TextureFormat) -> Self {
		let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("Skybox Shader"),
			source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/skybox.wgsl").into()),
		});
		let (buffer, bind_group, layout) = CameraUniform::get_buffer(device, 0);
		let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: Some("Skybox Pipeline Layout"),
			bind_group_layouts: &[Some(&layout)],
			immediate_size: 0,
		});
		let pipeline = WgpuWrapper::new(device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
			label: Some("Skybox Pipeline"),
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
					blend: Some(wgpu::BlendState::REPLACE),
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
	mut skybox: ResMut<SkyboxResource>,
	skybox_camera: Res<SkyboxCamera>,
	views: Query<&ViewTarget>,
) {
	let Some(view_target) = views.iter().next() else { return };
	skybox.ensure(render_device.wgpu_device(), view_target.main_texture_format());

	let Some(renderer) = skybox.renderer.as_ref() else { return };
	let Some((transform, projection)) = skybox_camera.0.as_ref() else { return };
	let Ok(uniform) = CameraUniform::from_camera(transform, projection) else { return };
	render_queue.write_buffer(&renderer.buffer, 0, uniform.as_bytes());
}

fn skybox_pass(
	world: &World,
	view: ViewQuery<&ViewTarget>,
	mut render_context: RenderContext,
) {
	let skybox = world.resource::<SkyboxResource>();
	let Some(renderer) = skybox.renderer.as_ref() else { return };

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
	pass.set_pipeline(&renderer.pipeline);
	pass.set_bind_group(0, &*renderer.bind_group, &[]);
	pass.draw(0..3, 0..1);
}
