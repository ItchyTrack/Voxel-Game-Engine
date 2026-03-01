use glam::{Vec3, Quat, Mat4};
use winit::keyboard::KeyCode;

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
		let view = Mat4::from_euler(glam::EulerRot::ZYX, 0.0, self.yaw, self.pitch).inverse() * Mat4::from_translation(self.position);
		let proj = Mat4::perspective_rh_gl(self.fovy, self.aspect, self.znear, self.zfar);
		return proj * view;
	}
}

pub struct CameraController {
	speed: f32,
	rotation_speed: f32,
	is_forward_pressed: bool,
	is_backward_pressed: bool,
	is_left_pressed: bool,
	is_right_pressed: bool,
	is_up_pressed: bool,
	is_down_pressed: bool,
	is_rotate_up_pressed: bool,
	is_rotate_down_pressed: bool,
	is_rotate_left_pressed: bool,
	is_rotate_right_pressed: bool,
}

impl CameraController {
	pub fn new(speed: f32, rotation_speed: f32) -> Self {
		Self {
			speed,
			rotation_speed,
			is_forward_pressed: false,
			is_backward_pressed: false,
			is_left_pressed: false,
			is_right_pressed: false,
			is_up_pressed: false,
			is_down_pressed: false,
			is_rotate_up_pressed: false,
			is_rotate_down_pressed: false,
			is_rotate_left_pressed: false,
			is_rotate_right_pressed: false,
		}
	}

	pub fn handle_key(&mut self, code: KeyCode, is_pressed: bool) -> bool {
		match code {
			KeyCode::KeyW => {
				self.is_forward_pressed = is_pressed;
				true
			}
			KeyCode::KeyS => {
				self.is_backward_pressed = is_pressed;
				true
			}
			KeyCode::KeyA => {
				self.is_left_pressed = is_pressed;
				true
			}
			KeyCode::KeyD => {
				self.is_right_pressed = is_pressed;
				true
			}
			KeyCode::KeyQ => {
				self.is_up_pressed = is_pressed;
				true
			}
			KeyCode::KeyE => {
				self.is_down_pressed = is_pressed;
				true
			}
			KeyCode::ArrowUp => {
				self.is_rotate_up_pressed = is_pressed;
				true
			}
			KeyCode::ArrowDown => {
				self.is_rotate_down_pressed = is_pressed;
				true
			}
			KeyCode::ArrowLeft => {
				self.is_rotate_left_pressed = is_pressed;
				true
			}
			KeyCode::ArrowRight => {
				self.is_rotate_right_pressed = is_pressed;
				true
			}
			_ => false,
		}
	}

	pub fn update_camera(&self, camera: &mut Camera) {
		if self.is_forward_pressed {
			camera.position += (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Z) * self.speed;
		}
		if self.is_backward_pressed {
			camera.position -= (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Z) * self.speed;
		}
		if self.is_left_pressed {
			camera.position += (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::X) * self.speed;
		}
		if self.is_right_pressed {
			camera.position -= (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::X) * self.speed;
		}
		if self.is_up_pressed {
			camera.position += (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Y) * self.speed;
		}
		if self.is_down_pressed {
			camera.position -= (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Y) * self.speed;
		}

		if self.is_rotate_up_pressed {
			camera.pitch += self.rotation_speed;
		}
		if self.is_rotate_down_pressed {
			camera.pitch -= self.rotation_speed;
		}
		if self.is_rotate_left_pressed {
			camera.yaw += self.rotation_speed;
		}
		if self.is_rotate_right_pressed {
			camera.yaw -= self.rotation_speed;
		}
	}
}
