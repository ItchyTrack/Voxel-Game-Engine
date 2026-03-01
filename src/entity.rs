use glam::{Mat3, Mat4, Quat, Vec3};
use crate::{camera, gpu_objects::mesh::{self, GetMesh}, voxels};

pub struct Entity {
	pub position: Vec3,
	pub orientation: Quat,
	pub momentum: Vec3,
	pub angular_momentum: Vec3,
	pub id: u32,
	pub voxels: voxels::Voxels,
	pub mesh: Option<mesh::Mesh>,
	pub update_mesh: bool,
}

impl Entity {
	pub fn update(&mut self) {
		self.position += self.momentum / self.voxels.mass();
		let com = self.voxels.center_of_mass();
		self.position += self.orientation * com;
		let rotational_inertia = Mat3::from_quat(self.orientation) * self.voxels.rotational_inertia() * Mat3::from_quat(self.orientation.inverse());
		let rotationa_velocity = rotational_inertia.inverse() * self.angular_momentum;
		self.orientation = Quat::from_scaled_axis(rotationa_velocity * (1./60.)) * self.orientation;
		self.position -= self.orientation * com;
	}

	pub fn get_rendering_meshes(&mut self, device: &wgpu::Device, camera: &camera::Camera) -> Vec<(&mesh::Mesh, Mat4)> {
		if self.update_mesh {
			self.mesh = self.voxels.get_mesh(device);
			self.update_mesh = false;
		}
		if self.mesh.is_some() {
			let matrix = Mat4::from_translation(self.position) * Mat4::from_quat(self.orientation);
			return vec![( self.mesh.as_ref().unwrap(), matrix )]
		}
		vec![]
	}
}
