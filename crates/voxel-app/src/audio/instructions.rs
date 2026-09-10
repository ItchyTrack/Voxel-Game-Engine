use voxel_math::FixedVec3;

#[derive(Clone, Copy, Debug)]
pub enum SoundEffect {
	BlockPlace,
	BlockBreak,
	DebugBeep,
}

#[derive(Clone, Copy)]
pub struct ListenerState {
	pub position: FixedVec3,
	pub forward: FixedVec3,
	pub right: FixedVec3,
}

impl Default for ListenerState {
	fn default() -> Self {
		Self {
			position: FixedVec3::ZERO,
			forward: FixedVec3::Z,
			right: FixedVec3::X
		}
	}
}

#[derive(Clone, Copy)]
pub struct SpawnVoiceInstruction {
	pub position: FixedVec3,
	pub frequency_hz: f32,
	pub gain: f32,
	pub max_volume_distance: f32,
	pub distance_falloff: f32,
	pub duration_seconds: f32,
	pub decay_rate: f32,
}

#[derive(Clone, Copy)]
pub enum AudioInstruction {
	SpawnVoice(SpawnVoiceInstruction),
	SetListener(ListenerState),
}
