use bevy::transform::components::Transform;
use super::math::{Mat6, Vec6};

pub const GAMMA: f32 = 0.99;

pub trait PhysicsConstraint {
	fn init(&mut self, _initial_state_1: &Transform, _initial_state_2: &Transform);
	fn get_updated(
			&self,
			state_1: &Transform,
			initial_state_1: &Transform,
			state_2: &Transform,
			initial_state_2: &Transform,
			alpha: f32,
			calc_1: bool) -> Option<(Vec6, Mat6)>;
	fn update_dual(
			&mut self,
			state_1: &Transform,
			initial_state_1: &Transform,
			state_2: &Transform,
			initial_state_2: &Transform,
			alpha: f32
		);
}
