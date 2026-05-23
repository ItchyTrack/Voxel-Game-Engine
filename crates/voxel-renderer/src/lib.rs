pub mod camera;
pub mod renderer;
pub mod crosshair_renderer;
pub mod debug_draw_renderer;
pub mod graphics_settings;
pub mod voxel_renderer;
pub mod debug_draw;

use bevy::{Update, prelude::*};

#[derive(Default)]
pub struct VoxelRendererPlugin;

impl Plugin for VoxelRendererPlugin{
	fn build(&self, app: &mut App) {
		app.add_systems(ScheduleLabel, systems)
	}
}


