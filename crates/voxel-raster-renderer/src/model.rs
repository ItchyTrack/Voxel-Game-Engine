use bevy::math::Mat4;
use bevy::render::render_resource::ShaderType;

#[derive(Debug, Copy, Clone, Default, ShaderType)]
pub struct ModelUniform {
	model: Mat4,
	palette_offset: u32,
}

impl ModelUniform {
	pub fn from_mat4(model: Mat4, palette_offset: u32) -> Self {
		Self { model, palette_offset }
	}
}
