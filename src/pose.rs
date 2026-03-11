use std::ops::*;

use glam::{Quat, Vec3};

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Pose {
	pub translation: Vec3,
	pub rotation: Quat,
}

impl Pose {
	pub const ZERO: Self = Self::new(Vec3::ZERO, Quat::IDENTITY);

	pub const fn new(translation: Vec3, rotation: Quat) -> Self {
		Self {
			translation,
			rotation,
		}
	}

	pub fn inverse(&self) -> Self {
		Self::new(self.rotation.inverse() * -self.translation, self.rotation.inverse())
	}
}

impl Mul<Pose> for Pose {
	type Output = Pose;
	#[inline]
	fn mul(self, rhs: Pose) -> Pose {
		Self::new(self.translation + self.rotation * rhs.translation, self.rotation * rhs.rotation)
	}
}

impl Mul<&Pose> for Pose {
	type Output = Pose;
	#[inline]
	fn mul(self, rhs: &Pose) -> Pose {
		self.mul(*rhs)
	}
}

impl Mul<Pose> for &Pose {
	type Output = Pose;
	#[inline]
	fn mul(self, rhs: Pose) -> Pose {
		(*self).mul(rhs)
	}
}

impl Mul<&Pose> for &Pose {
	type Output = Pose;
	#[inline]
	fn mul(self, rhs: &Pose) -> Pose {
		(*self).mul(*rhs)
	}
}

impl Mul<Vec3> for Pose {
	type Output = Vec3;
	#[inline]
	fn mul(self, rhs: Vec3) -> Vec3 {
		self.translation + self.rotation * rhs
	}
}

impl Mul<&Vec3> for Pose {
	type Output = Vec3;
	#[inline]
	fn mul(self, rhs: &Vec3) -> Vec3 {
		self.mul(*rhs)
	}
}

impl Mul<Vec3> for &Pose {
	type Output = Vec3;
	#[inline]
	fn mul(self, rhs: Vec3) -> Vec3 {
		(*self).mul(rhs)
	}
}

impl Mul<&Vec3> for &Pose {
	type Output = Vec3;
	#[inline]
	fn mul(self, rhs: &Vec3) -> Vec3 {
		(*self).mul(*rhs)
	}
}

impl Mul<Quat> for Pose {
	type Output = Quat;
	#[inline]
	fn mul(self, rhs: Quat) -> Quat {
		self.rotation * rhs
	}
}

impl Mul<&Quat> for Pose {
	type Output = Quat;
	#[inline]
	fn mul(self, rhs: &Quat) -> Quat {
		self.mul(*rhs)
	}
}

impl Mul<Quat> for &Pose {
	type Output = Quat;
	#[inline]
	fn mul(self, rhs: Quat) -> Quat {
		(*self).mul(rhs)
	}
}

impl Mul<&Quat> for &Pose {
	type Output = Quat;
	#[inline]
	fn mul(self, rhs: &Quat) -> Quat {
		(*self).mul(*rhs)
	}
}
