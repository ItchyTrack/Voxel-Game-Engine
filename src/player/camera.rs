use std::f32::consts::PI;

use glam::{Vec3, Quat, Mat4};
use winit::keyboard::KeyCode;

use crate::player;
use crate::pose::Pose;

pub struct ViewFrustum {
    planes: [Plane; 6],
}

struct Plane {
    normal: Vec3,
    distance_from_origin: f32,
}

impl Plane {
    fn signed_distance(&self, point: Vec3) -> f32 {
        self.normal.dot(point) - self.distance_from_origin
    }
}

impl ViewFrustum {
    pub fn compare_sphere(&self, center: Vec3, radius: f32) -> bool {
        self.planes.iter().all(|plane| {
            plane.signed_distance(center) >= -radius
        })
    }
}

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
		let view = Mat4::from_euler(glam::EulerRot::ZYX, 0.0, self.yaw + PI, -self.pitch).inverse() * Mat4::from_translation(-self.position);
		let proj = Mat4::perspective_rh_gl(self.fovy.to_radians(), self.aspect, self.znear, self.zfar);
		return proj * view;
	}

	pub fn quat(&self) -> Quat {
		Quat::from_euler(glam::EulerRot::ZYX, 0.0, self.yaw, self.pitch)
	}

	pub fn forward(&self) -> Vec3 {
		self.quat() * Vec3::Z
	}

	pub fn right(&self) -> Vec3 {
		self.quat() * -Vec3::X
	}

	pub fn up(&self) -> Vec3 {
		self.quat() * Vec3::Y
	}

	pub fn forward_right_up(&self) -> (Vec3, Vec3, Vec3) {
		let rotation = self.quat();
		(
			rotation * Vec3::Z,
			rotation * -Vec3::X,
			rotation * Vec3::Y
		)
	}

	pub fn pose(&self) -> Pose {
		Pose::new(self.position, self.quat())
	}

	pub fn frustum(&self) -> ViewFrustum {
		let (forward, right, up) = self.forward_right_up();

        let tan_fov = (self.fovy.to_radians() * 0.5).tan();
        let near_h = self.znear * tan_fov;
        let near_w = near_h * self.aspect;
        let far_h  = self.zfar  * tan_fov;
        let far_w  = far_h  * self.aspect;

        let near_center = self.position + forward * self.znear;
        let far_center = self.position + forward * self.zfar;

        let ntl = near_center + up * near_h - right * near_w;
        let ntr = near_center + up * near_h + right * near_w;
        let nbl = near_center - up * near_h - right * near_w;
        let nbr = near_center - up * near_h + right * near_w;
        let ftl = far_center + up * far_h  - right * far_w;
        let ftr = far_center + up * far_h  + right * far_w;
        let fbl = far_center - up * far_h  - right * far_w;
        // fbr unused

        let make_plane = |a: Vec3, b: Vec3, c: Vec3| {
            let normal = (b - a).cross(c - a).normalize();
            Plane { normal, distance_from_origin: normal.dot(a) }
        };

        ViewFrustum {
            planes: [
                make_plane(ntl, ntr, nbl), // near
                make_plane(ftr, ftl, fbl), // far
                make_plane(ntl, nbl, ftl), // left
                make_plane(ntr, ftr, nbr), // right
                make_plane(ntl, ftl, ntr), // top
                make_plane(nbl, nbr, fbl), // bottom
            ],
        }
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
		let unclamped_pitch= camera.pitch + dy as f32 * self.mouse_sensitivity;
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
			camera.position += camera.forward() * speed * dt;
		}
		if player_input.key(KeyCode::KeyS).is_pressed || player_input.key(KeyCode::KeyS).just_pressed {
			camera.position -= camera.forward() * speed * dt;
		}
		if player_input.key(KeyCode::KeyD).is_pressed || player_input.key(KeyCode::KeyD).just_pressed {
			camera.position += camera.right() * speed * dt;
		}
		if player_input.key(KeyCode::KeyA).is_pressed || player_input.key(KeyCode::KeyA).just_pressed {
			camera.position -= camera.right() * speed * dt;
		}
		if player_input.key(KeyCode::KeyQ).is_pressed || player_input.key(KeyCode::KeyQ).just_pressed {
			camera.position -= camera.up() * speed * dt;
		}
		if player_input.key(KeyCode::KeyE).is_pressed || player_input.key(KeyCode::KeyE).just_pressed {
			camera.position += camera.up() * speed * dt;
		}

		if player_input.key(KeyCode::ArrowUp).is_pressed || player_input.key(KeyCode::ArrowUp).just_pressed {
			camera.pitch -= self.rotation_speed * dt;
		}
		if player_input.key(KeyCode::ArrowDown).is_pressed || player_input.key(KeyCode::ArrowDown).just_pressed {
			camera.pitch += self.rotation_speed * dt;
		}
		if player_input.key(KeyCode::ArrowRight).is_pressed || player_input.key(KeyCode::ArrowRight).just_pressed {
			camera.yaw -= self.rotation_speed * dt;
		}
		if player_input.key(KeyCode::ArrowLeft).is_pressed || player_input.key(KeyCode::ArrowLeft).just_pressed {
			camera.yaw += self.rotation_speed * dt;
		}
	}
}
