use bevy::input::ButtonInput;
use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;

use camera_voxel_loader::FreezeCameraVoxelLoader;

pub struct DebugTogglesPlugin;

impl Plugin for DebugTogglesPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(Update, handle_debug_input);
	}
}

fn handle_debug_input(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	mut freeze: ResMut<FreezeCameraVoxelLoader>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if keys.just_pressed(KeyCode::KeyT) {
		freeze.0 = !freeze.0;
		info!("freeze_camera_voxel_loader = {}", freeze.0);
	}
}
