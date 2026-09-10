use crate::math::Wide;
use voxel_transform::Transform;
use crate::math::{Mat6, Vec6};

pub const GAMMA: Wide = Wide::from_bits(16_609_444i128 << 24);

pub trait PhysicsConstraint {
	fn init(&mut self, _initial_state_1: &Transform, _initial_state_2: &Transform);
	fn get_updated(
			&self,
			state_1: &Transform,
			initial_state_1: &Transform,
			state_2: &Transform,
			initial_state_2: &Transform,
			alpha: Wide,
			calc_1: bool) -> Option<(Vec6, Mat6)>;
	fn update_dual(
			&mut self,
			state_1: &Transform,
			initial_state_1: &Transform,
			state_2: &Transform,
			initial_state_2: &Transform,
			alpha: Wide
		);
}
