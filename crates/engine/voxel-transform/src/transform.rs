use std::ops::Mul;
use bevy::{math::{Affine3A, Mat3, Mat4, Quat, Vec3}, prelude::Component};
use fixed::traits::ToFixed;
use serde::{Deserialize, Serialize};
use voxel_math::{Fixed, FixedVec3};
use crate::Scale;

/// Authoritative local transform. Float transforms are rendering snapshots only.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Transform {
	pub translation: FixedVec3,
	pub rotation: Quat,
	pub scale: Scale,
}

impl Transform {
	pub const IDENTITY: Self = Self { translation: FixedVec3::ZERO, rotation: Quat::IDENTITY, scale: Scale::ONE };
	pub const fn from_translation(translation: FixedVec3) -> Self { Self { translation, ..Self::IDENTITY } }
	pub fn from_xyz<X: ToFixed, Y: ToFixed, Z: ToFixed>(x: X, y: Y, z: Z) -> Self {
		Self::from_translation(FixedVec3::new(Fixed::from_num(x), Fixed::from_num(y), Fixed::from_num(z)))
	}
	pub const fn from_rotation(rotation: Quat) -> Self { Self { rotation, ..Self::IDENTITY } }
	pub const fn from_scale(scale: Scale) -> Self { Self { scale, ..Self::IDENTITY } }
	pub const fn with_translation(mut self, translation: FixedVec3) -> Self { self.translation = translation; self }
	pub const fn with_rotation(mut self, rotation: Quat) -> Self { self.rotation = rotation; self }
	pub const fn with_scale(mut self, scale: Scale) -> Self { self.scale = scale; self }

	/// Point the local -Z axis along `direction`, keeping +Y near `up`.
	pub fn looking_to(mut self, direction: FixedVec3, up: FixedVec3) -> Self {
		let back = -direction.try_normalize().unwrap_or(FixedVec3::NEG_Z);
		let up = up.try_normalize().unwrap_or(FixedVec3::Y);
		let right = up.cross(back).try_normalize().unwrap_or_else(|| {
			FixedVec3::AXES[back.abs().min_position()].cross(back).normalize()
		});
		let up = back.cross(right).normalize();
		self.rotation = Quat::from_mat3(&Mat3::from_cols(right.as_vec3(), up.as_vec3(), back.as_vec3())).normalize();
		self
	}
	pub fn looking_at(self, target: FixedVec3, up: FixedVec3) -> Self { self.looking_to(target - self.translation, up) }
	pub fn forward(&self) -> FixedVec3 { self.rotation * FixedVec3::NEG_Z }
	pub fn back(&self) -> FixedVec3 { self.rotation * FixedVec3::Z }
	pub fn right(&self) -> FixedVec3 { self.rotation * FixedVec3::X }
	pub fn up(&self) -> FixedVec3 { self.rotation * FixedVec3::Y }

	pub fn inverse(&self) -> Self {
		let rotation = self.rotation.inverse();
		let scale = self.scale.recip();
		Self { translation: rotation * (-self.translation * scale.to_fixed()), rotation, scale }
	}
	pub fn transform_point(&self, point: FixedVec3) -> FixedVec3 { self.translation + self.transform_vector(point) }
	pub fn transform_vector(&self, vector: FixedVec3) -> FixedVec3 { self.rotation * (vector * self.scale.to_fixed()) }
	pub fn inverse_transform_point(&self, point: FixedVec3) -> FixedVec3 {
		voxel_math::Transform::inverse_transform_point(self, point)
	}
	pub fn inverse_transform_vector(&self, vector: FixedVec3) -> FixedVec3 {
		voxel_math::Transform::inverse_transform_vector(self, vector)
	}
	/// Returns None when the composed scale cannot be represented.
	pub fn checked_mul(&self, rhs: Self) -> Option<Self> {
		let scale = self.scale.checked_mul(rhs.scale)?;
		Some(Self { translation: self.transform_point(rhs.translation), rotation: self.rotation * rhs.rotation, scale })
	}

	/// Render conversion: subtract the shared origin before converting to floats.
	pub fn relative_to(&self, origin: FixedVec3) -> bevy::prelude::Transform {
		bevy::prelude::Transform {
			translation: (self.translation - origin).as_vec3(),
			rotation: self.rotation,
			scale: Vec3::splat(self.scale.to_fixed().to_num()),
		}
	}
	/// Render-only matrix conversion. Prefer `relative_to` for world positions.
	pub fn compute_matrix(&self) -> Mat4 { Mat4::from(self.as_affine()) }
	/// Render-only affine conversion. Prefer `relative_to` for world positions.
	pub fn as_affine(&self) -> Affine3A {
		Affine3A::from_scale_rotation_translation(Vec3::splat(self.scale.to_fixed().to_num()), self.rotation, self.translation.as_vec3())
	}
}

impl Default for Transform { fn default() -> Self { Self::IDENTITY } }
impl voxel_math::Transform for Transform {
	fn translation(&self) -> FixedVec3 { self.translation }
	fn rotation(&self) -> Quat { self.rotation }
	fn scale(&self) -> Fixed { self.scale.to_fixed() }
}
impl Mul for Transform {
	type Output = Self;
	fn mul(self, rhs: Self) -> Self { self.checked_mul(rhs).expect("transform scale composition underflow or overflow") }
}
impl Mul<FixedVec3> for Transform {
	type Output = FixedVec3;
	fn mul(self, rhs: FixedVec3) -> FixedVec3 { self.transform_point(rhs) }
}
