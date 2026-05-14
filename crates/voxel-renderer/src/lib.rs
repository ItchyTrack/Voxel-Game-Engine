pub mod renderer;
pub mod crosshair_renderer;
pub mod debug_draw_renderer;
pub mod graphics_settings;
pub mod voxel_renderer;

use bevy::prelude::*;

#[derive(Default)]
pub struct VoxelRendererPlugin;

impl Plugin for VoxelRendererPlugin{
	fn build(&self, _app: &mut App) {

	}
}
