use bevy::math::Mat4;
use bevy::render::render_resource::ShaderType;

#[derive(Debug, Copy, Clone, Default, ShaderType)]
pub struct ModelUniform {
	model: Mat4,
}

impl ModelUniform {
	pub fn from_mat4(model: Mat4) -> Self { Self { model } }
}
