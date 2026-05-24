use bevy::ecs::query::QueryItem;
use bevy::ecs::system::{Res, ResMut};
use bevy::ecs::world::World;
use bevy::render::render_graph::{
	NodeRunError, RenderGraphContext, RenderLabel, ViewNode,
};
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue};
use bevy::render::view::ViewTarget;

use crate::crosshair_renderer::CrosshairResource;

#[derive(Debug, Hash, PartialEq, Eq, Clone, Copy, RenderLabel)]
pub struct CrosshairRenderLabel;

pub fn prepare_crosshair(
	render_device: Res<RenderDevice>,
	render_queue: Res<RenderQueue>,
	mut crosshair: ResMut<CrosshairResource>,
	views: bevy::ecs::system::Query<&ViewTarget>,
) {
	let Some(view_target) = views.iter().next() else { return };
	crosshair.ensure(render_device.wgpu_device(), view_target.main_texture_format());

	let Some(renderer) = crosshair.renderer.as_ref() else { return };
	let main_texture = view_target.main_texture();
	render_queue.write_buffer(
		&renderer.buffer,
		0,
		bytemuck::cast_slice(&[main_texture.width() as f32, main_texture.height() as f32]),
	);
}

#[derive(Default)]
pub struct CrosshairRenderNode;

impl ViewNode for CrosshairRenderNode {
	type ViewQuery = &'static ViewTarget;

	fn run<'w>(
		&self,
		_graph: &mut RenderGraphContext,
		render_context: &mut RenderContext<'w>,
		view_target: QueryItem<'w, '_, Self::ViewQuery>,
		world: &'w World,
	) -> Result<(), NodeRunError> {
		let crosshair = world.resource::<CrosshairResource>();
		let Some(renderer) = crosshair.renderer.as_ref() else { return Ok(()) };

		let encoder = render_context.command_encoder();
		let view = view_target.main_texture_view();

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
		});
		pass.set_pipeline(&renderer.pipeline);
		pass.set_bind_group(0, &renderer.bind_group, &[]);
		pass.draw(0..3, 0..1);

		Ok(())
	}
}
