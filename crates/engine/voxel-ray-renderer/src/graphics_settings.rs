use bevy::ecs::resource::Resource;
use bevy::math::UVec4;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::ShaderType;

#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LightingMode {
	#[default]
	None = 0,
	Direct = 1,
	Indirect = 2,
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GiRayCount {
	Eight = 8,
	Sixteen = 16,
	#[default]
	ThirtyTwo = 32,
	SixtyFour = 64,
}

impl GiRayCount {
	pub fn count(self) -> u32 { self as u32 }
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DirectRayCount {
	#[default]
	One = 1,
	Four = 4,
	Nine = 9,
}

impl DirectRayCount {
	pub fn count(self) -> u32 { self as u32 }
}

#[derive(Resource, ExtractResource, Clone, Copy, Debug, Default)]
pub struct GraphicsSettings {
	pub lighting: LightingMode,
	pub anti_aliasing: bool,
	pub gi_rays: GiRayCount,
	pub direct_rays: DirectRayCount,
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
		Self { values: UVec4::new(settings.lighting as u32, settings.anti_aliasing as u32, settings.direct_rays.count(), 0) }
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn lighting_is_opt_in_with_thirty_two_rays() {
		let settings = GraphicsSettings::default();
		assert_eq!(settings.lighting, LightingMode::None);
		assert_eq!(settings.gi_rays, GiRayCount::ThirtyTwo);
		assert_eq!(settings.gi_rays.count(), 32);
		assert_eq!(settings.direct_rays, DirectRayCount::One);
		assert_eq!(RenderSettingsUniform::from_graphics_settings(&settings).values, UVec4::new(0, 0, 1, 0));
	}

	#[test]
	fn enum_values_match_shader_contract() {
		assert_eq!([LightingMode::None as u32, LightingMode::Direct as u32, LightingMode::Indirect as u32], [0, 1, 2]);
		assert_eq!([GiRayCount::Eight, GiRayCount::Sixteen, GiRayCount::ThirtyTwo, GiRayCount::SixtyFour].map(GiRayCount::count), [8, 16, 32, 64]);
		assert_eq!([DirectRayCount::One, DirectRayCount::Four, DirectRayCount::Nine].map(DirectRayCount::count), [1, 4, 9]);
	}

	#[test]
	fn uniform_flags_are_independent_of_ray_count() {
		for lighting in [LightingMode::None, LightingMode::Direct, LightingMode::Indirect] {
			for anti_aliasing in [false, true] {
				for gi_rays in [GiRayCount::Eight, GiRayCount::Sixteen, GiRayCount::ThirtyTwo, GiRayCount::SixtyFour] {
					for direct_rays in [DirectRayCount::One, DirectRayCount::Four, DirectRayCount::Nine] {
						let settings = GraphicsSettings { lighting, anti_aliasing, gi_rays, direct_rays };
						let uniform = RenderSettingsUniform::from_graphics_settings(&settings);
						assert_eq!(uniform.values.to_array(), [lighting as u32, anti_aliasing as u32, direct_rays.count(), 0]);
					}
				}
			}
		}
	}
}
