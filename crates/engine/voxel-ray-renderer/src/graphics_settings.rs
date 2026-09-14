use bevy::ecs::resource::Resource;
use bevy::math::UVec4;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::ShaderType;

#[derive(Resource, ExtractResource, Clone, Copy, Debug)]
pub struct GraphicsSettings {
	pub shadows: bool,
	pub anti_aliasing: bool,
	pub face_gi: bool,
}

impl Default for GraphicsSettings {
	fn default() -> Self {
		Self { shadows: false, anti_aliasing: false, face_gi: false }
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
		Self { values: UVec4::new(settings.shadows as u32, settings.anti_aliasing as u32, settings.face_gi as u32, 0) }
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn face_gi_is_opt_in() {
		let settings = GraphicsSettings::default();
		assert!(!settings.face_gi);
		assert_eq!(RenderSettingsUniform::from_graphics_settings(&settings).values, UVec4::ZERO);
	}

	#[test]
	fn uniform_flags_are_independent() {
		for shadows in [false, true] {
			for anti_aliasing in [false, true] {
				for face_gi in [false, true] {
					let settings = GraphicsSettings { shadows, anti_aliasing, face_gi };
					let uniform = RenderSettingsUniform::from_graphics_settings(&settings);
					assert_eq!(uniform.values.to_array(), [shadows as u32, anti_aliasing as u32, face_gi as u32, 0]);
				}
			}
		}
	}
}
