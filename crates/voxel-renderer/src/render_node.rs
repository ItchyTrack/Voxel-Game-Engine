use bevy::ecs::query::QueryItem;
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Res, ResMut};
use bevy::ecs::world::World;
use bevy::render::render_graph::{
	NodeRunError, RenderGraphContext, RenderLabel, ViewNode,
};
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue};
use bevy::render::view::ViewTarget;

use crate::camera::CameraUniform;
use crate::extract::ExtractedVoxelScene;
use crate::graphics_settings::{GraphicsSettings, RenderSettingsUniform};
use crate::hit_count_feedback::LastGpuBvh;
use crate::voxel_renderer::VoxelRendererResource;

#[derive(Debug, Hash, PartialEq, Eq, Clone, Copy, RenderLabel)]
pub struct VoxelRenderLabel;

#[derive(Resource, Default)]
pub struct VoxelViewBindGroups {
	pub ready: bool,
}

pub fn prepare_voxel_view_bind_groups(
	render_device: Res<RenderDevice>,
	render_queue: Res<RenderQueue>,
	extracted: Res<ExtractedVoxelScene>,
	mut voxel_resource: ResMut<VoxelRendererResource>,
	mut bind_groups: ResMut<VoxelViewBindGroups>,
	views: bevy::ecs::system::Query<&ViewTarget>,
) {
	bind_groups.ready = false;

	if !extracted.has_camera { return; }
	let Some(projection) = extracted.camera_projection.as_ref() else { return };

	let Some(view_target) = views.iter().next() else { return };
	let main_texture = view_target.main_texture();
	let width = main_texture.width();
	let height = main_texture.height();
	let format = view_target.main_texture_format();

	voxel_resource.ensure(render_device.wgpu_device(), width, height, format);
	if voxel_resource.voxel_renderer.is_none() { return; }

	let Ok(cam_uniform) = CameraUniform::from_camera(&extracted.camera_transform, projection) else {
		log::warn!("voxel renderer: unsupported camera projection (expected perspective)");
		return;
	};

	render_queue.write_buffer(&voxel_resource.camera_buffer, 0, bytemuck::bytes_of(&cam_uniform));
	let settings = RenderSettingsUniform::from_graphics_settings(&GraphicsSettings::new());
	render_queue.write_buffer(&voxel_resource.render_settings_buffer, 0, bytemuck::bytes_of(&settings));

	bind_groups.ready = true;
}

#[derive(Default)]
pub struct VoxelRenderNode;

impl ViewNode for VoxelRenderNode {
	type ViewQuery = &'static ViewTarget;

	fn run<'w>(
		&self,
		_graph: &mut RenderGraphContext,
		render_context: &mut RenderContext<'w>,
		view_target: QueryItem<'w, '_, Self::ViewQuery>,
		world: &'w World,
	) -> Result<(), NodeRunError> {
		if !world.resource::<VoxelViewBindGroups>().ready { return Ok(()); }

		let voxel_resource = world.resource::<VoxelRendererResource>();
		let Some(voxel_renderer) = voxel_resource.voxel_renderer.as_ref() else { return Ok(()) };

		let extracted = world.resource::<ExtractedVoxelScene>();
		let Some(bvh) = extracted.bvh.as_ref() else { return Ok(()) };
		if extracted.id_to_offsets.is_empty() { return Ok(()) }
		let (Some(tree_buffer), Some(voxel_buffer)) =
			(extracted.tree_buffer.as_ref(), extracted.voxel_buffer.as_ref())
		else { return Ok(()) };

		let device = render_context.render_device().wgpu_device().clone();
		let encoder = render_context.command_encoder();
		let main_texture = view_target.main_texture();
		let color_attachment = view_target.get_color_attachment();

		let gpu_bvh = voxel_renderer.render(
			&device,
			encoder,
			main_texture.width(),
			main_texture.height(),
			&voxel_resource.camera_bind_group,
			bvh,
			&extracted.id_to_offsets,
			tree_buffer,
			voxel_buffer,
			color_attachment,
		);

		// Hold onto the GpuBvh so next frame's prepare can read back its
		// item-hit-count staging buffer.
		if let Ok(mut slot) = world.resource::<LastGpuBvh>().0.lock() {
			*slot = Some(gpu_bvh);
		}

		Ok(())
	}
}
