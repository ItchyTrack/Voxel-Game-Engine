use bevy::math::Mat4;
use bevy::render::render_resource::ShaderType;
use bevy::render::view::ExtractedView;

#[derive(Debug, Copy, Clone, Default, ShaderType)]
pub struct CameraUniform {
	view_proj: Mat4,
}

impl CameraUniform {
	pub fn from_view(view: &ExtractedView) -> Self {
		let view_proj = view.clip_from_world.unwrap_or_else(|| {
			view.clip_from_view * view.world_from_view.to_matrix().inverse()
		});
		Self { view_proj }
	}
}
