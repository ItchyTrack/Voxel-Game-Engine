use glam::{Vec3, Quat, Mat4};
use winit::keyboard::KeyCode;

use crate::player;

pub struct Camera {
	pub position: Vec3,
	pub yaw: f32,
	pub pitch: f32,
	pub aspect: f32,
	pub fovy: f32,
	pub znear: f32,
	pub zfar: f32,
}

impl Camera {
	pub fn build_view_projection_matrix(&self) -> Mat4 {
		let view = Mat4::from_euler(glam::EulerRot::ZYX, 0.0, self.yaw, self.pitch).inverse() * Mat4::from_translation(-self.position);
		let proj = Mat4::perspective_rh_gl(self.fovy, self.aspect, self.znear, self.zfar);
		return proj * view;
	}
}

pub struct CameraController {
	speed: f32,
	rotation_speed: f32,
	mouse_sensitivity: f32,
}

impl CameraController {
	pub fn new(speed: f32, rotation_speed: f32, mouse_sensitivity: f32) -> Self {
		Self {
			speed,
			rotation_speed,
			mouse_sensitivity
		}
	}

	pub fn handle_mouse_motion(&self, camera: &mut Camera, dx: f64, dy: f64) {
		camera.yaw -= dx as f32 * self.mouse_sensitivity;
		let unclamped_pitch= camera.pitch - dy as f32 * self.mouse_sensitivity;
		camera.pitch = unclamped_pitch.clamp(
			-std::f32::consts::FRAC_PI_2 + 0.01,
			std::f32::consts::FRAC_PI_2 - 0.01
		);
	}

	pub fn update_camera(&self, camera: &mut Camera, player_input: &player::player_input::PlayerInput, dt: f32) {
		let speed = self.speed * if
			player_input.key(KeyCode::ShiftLeft).is_pressed || player_input.key(KeyCode::ShiftLeft).just_pressed ||
			player_input.key(KeyCode::ShiftRight).is_pressed || player_input.key(KeyCode::ShiftRight).just_pressed
		{ 4.0 } else { 1.0 };
		if player_input.key(KeyCode::KeyW).is_pressed || player_input.key(KeyCode::KeyW).just_pressed {
			camera.position -= (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Z) * speed * dt;
		}
		if player_input.key(KeyCode::KeyS).is_pressed || player_input.key(KeyCode::KeyS).just_pressed {
			camera.position += (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Z) * speed * dt;
		}
		if player_input.key(KeyCode::KeyA).is_pressed || player_input.key(KeyCode::KeyA).just_pressed {
			camera.position -= (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::X) * speed * dt;
		}
		if player_input.key(KeyCode::KeyD).is_pressed || player_input.key(KeyCode::KeyD).just_pressed {
			camera.position += (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::X) * speed * dt;
		}
		if player_input.key(KeyCode::KeyQ).is_pressed || player_input.key(KeyCode::KeyQ).just_pressed {
			camera.position -= (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Y) * speed * dt;
		}
		if player_input.key(KeyCode::KeyE).is_pressed || player_input.key(KeyCode::KeyE).just_pressed {
			camera.position += (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Y) * speed * dt;
		}

		if player_input.key(KeyCode::ArrowUp).is_pressed || player_input.key(KeyCode::ArrowUp).just_pressed {
			camera.pitch += self.rotation_speed * dt;
		}
		if player_input.key(KeyCode::ArrowDown).is_pressed || player_input.key(KeyCode::ArrowDown).just_pressed {
			camera.pitch -= self.rotation_speed * dt;
		}
		if player_input.key(KeyCode::ArrowLeft).is_pressed || player_input.key(KeyCode::ArrowLeft).just_pressed {
			camera.yaw += self.rotation_speed * dt;
		}
		if player_input.key(KeyCode::ArrowRight).is_pressed || player_input.key(KeyCode::ArrowRight).just_pressed {
			camera.yaw -= self.rotation_speed * dt;
		}
	}
}
