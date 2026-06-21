use bevy::input::ButtonInput;
use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::input::InputSystems;
use bevy::math::{Quat, Vec3};
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, CursorOptions, PrimaryWindow};
use bevy_egui::input::EguiWantsInput;

#[derive(Component)]
pub struct FlyCamera {
	pub speed: f32,
	pub rotation_speed: f32,
	pub mouse_sensitivity: f32,
	pub yaw: f32,
	pub pitch: f32,
}

impl Default for FlyCamera {
	fn default() -> Self {
		Self {
			speed: 30.0,
			rotation_speed: 1.5,
			mouse_sensitivity: 0.0015,
			yaw: 0.0,
			pitch: 0.0,
		}
	}
}

pub struct FlyCameraPlugin;

impl Plugin for FlyCameraPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(PreUpdate, fly_camera_system.after(InputSystems))
			.add_systems(Update, toggle_cursor_grab);
	}
}

const PITCH_LIMIT: f32 = std::f32::consts::FRAC_PI_2 - 0.01;

fn fly_camera_system(
	time: Res<Time>,
	keys: Res<ButtonInput<KeyCode>>,
	mouse_motion: Res<AccumulatedMouseMotion>,
	cursor_options: Query<&CursorOptions, With<PrimaryWindow>>,
	mut cams: Query<(&mut Transform, &mut FlyCamera)>,
) {
	let dt = time.delta_secs();

	let mouse_delta = if cfg!(target_arch = "wasm32") {
		mouse_motion.delta
	} else {
		let mouse_captured = cursor_options
			.single()
			.map(|c| c.grab_mode != CursorGrabMode::None)
			.unwrap_or(false);
		if mouse_captured { mouse_motion.delta } else { Vec2::ZERO }
	};

	for (mut transform, mut cam) in cams.iter_mut() {
		cam.yaw   -= mouse_delta.x * cam.mouse_sensitivity;
		cam.pitch -= mouse_delta.y * cam.mouse_sensitivity;

		if keys.pressed(KeyCode::ArrowLeft)  { cam.yaw   += cam.rotation_speed * dt; }
		if keys.pressed(KeyCode::ArrowRight) { cam.yaw   -= cam.rotation_speed * dt; }
		if keys.pressed(KeyCode::ArrowUp)    { cam.pitch += cam.rotation_speed * dt; }
		if keys.pressed(KeyCode::ArrowDown)  { cam.pitch -= cam.rotation_speed * dt; }

		cam.pitch = cam.pitch.clamp(-PITCH_LIMIT, PITCH_LIMIT);

		transform.rotation = Quat::from_axis_angle(Vec3::Y, cam.yaw)
			* Quat::from_axis_angle(Vec3::X, cam.pitch);

		let forward = transform.forward().as_vec3();
		let right = transform.right().as_vec3();
		let up = transform.up().as_vec3();

		let sprint = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
		let speed = cam.speed * if sprint { 16.0 } else { 1.0 };

		let mut move_dir = Vec3::ZERO;
		if keys.pressed(KeyCode::KeyW) { move_dir += forward; }
		if keys.pressed(KeyCode::KeyS) { move_dir -= forward; }
		if keys.pressed(KeyCode::KeyD) { move_dir += right; }
		if keys.pressed(KeyCode::KeyA) { move_dir -= right; }
		if keys.pressed(KeyCode::KeyE) { move_dir += up; }
		if keys.pressed(KeyCode::KeyQ) { move_dir -= up; }

		if move_dir != Vec3::ZERO {
			transform.translation += move_dir * speed * dt;
		}
	}
}

fn toggle_cursor_grab(
	mouse_buttons: Res<ButtonInput<MouseButton>>,
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	mut cursor_options: Query<&mut CursorOptions, With<PrimaryWindow>>,
) {
	let egui_pointer = egui_wants.as_ref().is_some_and(|e| e.wants_any_pointer_input());
	let egui_keys = egui_wants.as_ref().is_some_and(|e| e.wants_any_keyboard_input());
	let Ok(mut cursor) = cursor_options.single_mut() else { return };
	if mouse_buttons.just_pressed(MouseButton::Left) && !egui_pointer {
		cursor.grab_mode = CursorGrabMode::Locked;
		cursor.visible = false;
	} else if keys.just_pressed(KeyCode::Escape) && !egui_keys {
		cursor.grab_mode = CursorGrabMode::None;
		cursor.visible = true;
	}
}
