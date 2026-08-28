use bevy::ecs::resource::Resource;
use bevy::math::UVec4;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::ShaderType;

#[derive(Resource, ExtractResource, Clone, Copy, Debug)]
pub struct GraphicsSettings {
	pub shadows: bool,
	pub anti_aliasing: bool,
}

impl Default for GraphicsSettings {
	fn default() -> Self {
		Self { shadows: false, anti_aliasing: false }
	}
}

impl GraphicsSettings {
	pub fn new() -> Self {
		Self::default()
	}
}

#[derive(Debug, Copy, Clone, Default, ShaderType)]
pub struct RenderSettingsUniform {
	values: UVec4,
}

impl RenderSettingsUniform {
	pub fn from_graphics_settings(settings: &GraphicsSettings) -> Self {
		Self { values: UVec4::new(settings.shadows as u32, settings.anti_aliasing as u32, 0, 0) }
	}
}
