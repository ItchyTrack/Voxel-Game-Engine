pub struct GraphicsSettings {
	pub shadows: bool,
}

impl GraphicsSettings {
	pub fn new() -> Self {
		Self { shadows: false }
	}
}

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct RenderSettingsUniform {
	values: [u32; 4],
}

impl RenderSettingsUniform {
	pub fn from_graphics_settings(settings: &GraphicsSettings) -> Self {
		Self { values: [settings.shadows as u32, 0, 0, 0] }
	}
}
