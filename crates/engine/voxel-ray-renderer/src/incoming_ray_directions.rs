bitflags::bitflags! {
	#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
	pub struct IncomingRayDirections: u8 {
		const NEG_X = 1 << 0;
		const POS_X = 1 << 1;
		const NEG_Y = 1 << 2;
		const POS_Y = 1 << 3;
		const NEG_Z = 1 << 4;
		const POS_Z = 1 << 5;
	}
}
