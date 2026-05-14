use super::super::pose::Pose;
use super::math::{Mat6, Vec6};

pub const GAMMA: f32 = 0.99;

pub trait PhysicsConstraint {
	fn init(&mut self, _initial_state_1: &Pose, _initial_state_2: &Pose);
	fn get_updated(
			&self,
			state_1: &Pose,
			initial_state_1: &Pose,
			state_2: &Pose,
			initial_state_2: &Pose,
			alpha: f32,
			calc_1: bool) -> Option<(Vec6, Mat6)>;
	fn update_dual(
			&mut self,
			state_1: &Pose,
			initial_state_1: &Pose,
			state_2: &Pose,
			initial_state_2: &Pose,
			alpha: f32
		);
}
