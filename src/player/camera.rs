use std::f32::consts::PI;

use glam::{Mat4, Quat, Vec3};
use winit::keyboard::KeyCode;

use crate::player;

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
	pub fn build_transform_matrix(&self) -> Mat4 {
		Mat4::from_translation(self.position) * Mat4::from_euler(glam::EulerRot::ZYX, 0.0, self.yaw, self.pitch)
	}

	pub fn build_view_projection_matrix(&self) -> Mat4 {
		let view = Mat4::from_euler(glam::EulerRot::ZYX, 0.0, self.yaw + PI, -self.pitch).inverse() * Mat4::from_translation(-self.position);
		let proj = Mat4::perspective_rh_gl(self.fovy.to_radians(), self.aspect, self.znear, self.zfar);
		return proj * view;
	}

	pub fn frustum(&self) -> ViewFrustum {
		let forward = Quat::from_euler(glam::EulerRot::ZYX, 0.0, self.yaw, self.pitch) * Vec3::Z;
		let right = forward.cross(Vec3::Y).normalize();
		let up = right.cross(forward).normalize();

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
			camera.position += (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Z) * speed * dt;
		}
		if player_input.key(KeyCode::KeyS).is_pressed || player_input.key(KeyCode::KeyS).just_pressed {
			camera.position -= (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Z) * speed * dt;
		}
		if player_input.key(KeyCode::KeyA).is_pressed || player_input.key(KeyCode::KeyA).just_pressed {
			camera.position += (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::X) * speed * dt;
		}
		if player_input.key(KeyCode::KeyD).is_pressed || player_input.key(KeyCode::KeyD).just_pressed {
			camera.position -= (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::X) * speed * dt;
		}
		if player_input.key(KeyCode::KeyQ).is_pressed || player_input.key(KeyCode::KeyQ).just_pressed {
			camera.position -= (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Y) * speed * dt;
		}
		if player_input.key(KeyCode::KeyE).is_pressed || player_input.key(KeyCode::KeyE).just_pressed {
			camera.position += (Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch) * Vec3::Y) * speed * dt;
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


#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
	transform: [[f32; 4]; 4],
	camera_view_size: [f32; 2],
	padding: [u8; 8],
}

impl CameraUniform {
	pub fn from_camera(camera: &Camera) -> Self {
		let tan_fov = (camera.fovy.to_radians() * 0.5).tan();
		Self {
			transform: camera.build_transform_matrix().to_cols_array_2d(),
			camera_view_size: [
				tan_fov * camera.aspect,
				tan_fov,
			],
			padding: [0; 8],
		}
	}

	pub fn get_bind_group_layout(device: &wgpu::Device, binding: u32) -> wgpu::BindGroupLayout {
		device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
				entries: &[wgpu::BindGroupLayoutEntry {
					binding: binding,
					visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: None,
					},
					count: None,
				}],
				label: Some("camera_bind_group_layout"),
			})
	}
	pub fn get_dynamic_offset_bind_group_layout(device: &wgpu::Device, binding: u32) -> wgpu::BindGroupLayout {
		device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
				entries: &[wgpu::BindGroupLayoutEntry {
					binding: binding,
					visibility: wgpu::ShaderStages::VERTEX,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: true,
						min_binding_size: None,
					},
					count: None,
				}],
				label: Some("camera_bind_group_layout"),
			})
	}
	pub fn get_buffer(device: &wgpu::Device, binding: u32) -> (wgpu::Buffer, wgpu::BindGroup, wgpu::BindGroupLayout) {
		let buffer = device.create_buffer(&wgpu::BufferDescriptor {
				label: Some("camera_buffer"),
				size: std::mem::size_of::<CameraUniform>() as u64,
				usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
				mapped_at_creation: false,
			});
		let bind_group_layout = Self::get_bind_group_layout(&device, binding);
		let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &bind_group_layout,
				entries: &[wgpu::BindGroupEntry {
					binding: binding,
					resource: buffer.as_entire_binding(),
				}],
				label: Some("camera_bind_group"),
			});
		(buffer, bind_group, bind_group_layout)
	}
}
