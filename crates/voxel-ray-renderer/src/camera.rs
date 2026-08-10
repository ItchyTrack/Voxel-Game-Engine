use bevy::math::{Mat4, Vec2};
use bevy::render::render_resource::ShaderType;
use bevy::render::view::ExtractedView;

#[derive(Debug, Copy, Clone, Default, ShaderType)]
pub struct CameraUniform {
	transform: Mat4,
	world_from_clip: Mat4,
	camera_view_size: Vec2,
}

impl CameraUniform {
	pub fn from_view(view: &ExtractedView) -> Result<Self, ()> {
		if view.clip_from_view.w_axis.w != 0.0 { return Err(()); }
		Ok(Self {
			transform: view.world_from_view.to_matrix(),
			world_from_clip: view.clip_from_world
				.unwrap_or_else(|| view.clip_from_view * view.world_from_view.to_matrix().inverse())
				.inverse(),
			camera_view_size: Vec2::new(
				view.clip_from_view.x_axis.x.recip(),
				view.clip_from_view.y_axis.y.recip(),
			),
		})
	}
}
