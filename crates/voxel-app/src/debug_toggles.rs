use bevy::input::ButtonInput;
use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;

use voxel_renderer::scene::FreezeUploads;

pub struct DebugTogglesPlugin;

impl Plugin for DebugTogglesPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<FreezeUploads>()
			.add_systems(Update, handle_debug_input);
	}
}

fn handle_debug_input(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	mut freeze: ResMut<FreezeUploads>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if keys.just_pressed(KeyCode::KeyT) {
		freeze.0 = !freeze.0;
		info!("freeze_gpu_grids = {}", freeze.0);
	}
}
